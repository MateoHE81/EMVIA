import http from "node:http";
import process from "node:process";
import { WebSocket, WebSocketServer } from "ws";

import { config } from "./config.js";
import {
  parseJsonMessage,
  RelayProtocolError,
  validateEmotionMessage,
  validateSubscribeMessage,
  validateUnsubscribeMessage
} from "./emotionMessage.js";
import { MatchRoomManager } from "./matchRoomManager.js";
import {
  ConnectionManager,
  requestHasValidToken
} from "./connectionManager.js";

const startedUtcMs = Date.now();
const roomManager = new MatchRoomManager({
  cacheWindowMs: config.cacheWindowMs,
  maxCachedMessagesPerMatch: config.maxCachedMessagesPerMatch,
  maxHomeClientsPerMatch: config.maxHomeClientsPerMatch
});
const connectionManager = new ConnectionManager({
  maxMessagesPerSecond: config.maxMessagesPerSecond
});

const server = http.createServer((request, response) => {
  const requestURL = new URL(request.url, `http://${request.headers.host || "relay.local"}`);

  if (request.method === "GET" && requestURL.pathname === "/health") {
    const body = JSON.stringify({
      status: "ok",
      service: "stadium-emotion-relay",
      protocolVersion: config.protocolVersion,
      serverUtcMs: Date.now(),
      uptimeMs: Date.now() - startedUtcMs,
      rooms: roomManager.summaries(),
      homeClients: roomManager.totalHomeClients,
      stadiumClients: roomManager.totalStadiumClients
    });

    response.writeHead(200, {
      "content-type": "application/json; charset=utf-8",
      "cache-control": "no-store"
    });
    response.end(body);
    return;
  }

  response.writeHead(404, { "content-type": "application/json; charset=utf-8" });
  response.end(JSON.stringify({ error: "Not found" }));
});

const webSocketServer = new WebSocketServer({
  noServer: true,
  maxPayload: config.maxPayloadBytes,
  perMessageDeflate: false,
  clientTracking: true
});

function sendJSON(socket, message) {
  if (socket.readyState === WebSocket.OPEN) {
    socket.send(JSON.stringify(message));
  }
}

function sendError(socket, error) {
  const code = error instanceof RelayProtocolError
    ? error.code
    : "SERVER_ERROR";

  sendJSON(socket, {
    type: "error",
    protocolVersion: config.protocolVersion,
    code,
    message: error.message,
    serverUtcMs: Date.now()
  });
}

function handleClientMessage(socket, rawData, isBinary) {
  if (!connectionManager.allowMessage(socket)) {
    sendError(
      socket,
      new RelayProtocolError(
        "RATE_LIMITED",
        `Maximum rate is ${config.maxMessagesPerSecond} messages per second.`
      )
    );
    return;
  }

  try {
    const value = parseJsonMessage(rawData, isBinary);

    switch (value.type) {
      case "emotion": {
        const message = validateEmotionMessage(
          value,
          config.protocolVersion
        );

        const wasUnknown = socket.relayContext.role === "unknown";
        const result = roomManager.publishEmotion(socket, message);

        if (wasUnknown) {
          sendJSON(socket, {
            type: "publisherReady",
            protocolVersion: config.protocolVersion,
            role: "stadium",
            matchID: message.matchID,
            deviceID: message.deviceID,
            serverUtcMs: Date.now()
          });
        }

        if (result.duplicate) {
          sendError(
            socket,
            new RelayProtocolError(
              "DUPLICATE_PACKET",
              "This device sequence and event timestamp were already relayed."
            )
          );
        }
        break;
      }

      case "subscribe": {
        const subscription = validateSubscribeMessage(
          value,
          config.protocolVersion
        );
        const result = roomManager.subscribeHome(socket, subscription);

        sendJSON(socket, {
          type: "subscribed",
          protocolVersion: config.protocolVersion,
          role: "home",
          matchID: result.matchID,
          homeClients: result.homeClients,
          replayed: result.replayed,
          serverUtcMs: Date.now()
        });
        break;
      }

      case "unsubscribe": {
        validateUnsubscribeMessage(value, config.protocolVersion);
        const previousMatchID = socket.relayContext.matchID;
        roomManager.removeSocket(socket);

        sendJSON(socket, {
          type: "unsubscribed",
          protocolVersion: config.protocolVersion,
          matchID: previousMatchID,
          serverUtcMs: Date.now()
        });
        break;
      }

      default:
        throw new RelayProtocolError(
          "UNKNOWN_MESSAGE_TYPE",
          `Unsupported message type: ${String(value.type)}`
        );
    }

    connectionManager.resetProtocolErrors(socket);
  } catch (error) {
    const errorCount = connectionManager.recordProtocolError(socket);
    sendError(socket, error instanceof Error ? error : new Error("Unknown error"));

    if (errorCount >= 5) {
      socket.close(1008, "Too many protocol errors");
    }
  }
}

server.on("upgrade", (request, socket, head) => {
  const requestURL = new URL(request.url, "http://relay.local");

  if (requestURL.pathname !== config.websocketPath) {
    socket.write("HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\n");
    socket.destroy();
    return;
  }

  if (!requestHasValidToken(request, config.relayToken)) {
    socket.write("HTTP/1.1 401 Unauthorized\r\nConnection: close\r\n\r\n");
    socket.destroy();
    return;
  }

  webSocketServer.handleUpgrade(request, socket, head, (webSocket) => {
    webSocketServer.emit("connection", webSocket, request);
  });
});

webSocketServer.on("connection", (socket, request) => {
  const context = connectionManager.initialize(socket, request);

  sendJSON(socket, {
    type: "hello",
    protocolVersion: config.protocolVersion,
    connectionID: context.connectionID,
    serverUtcMs: Date.now(),
    websocketPath: config.websocketPath
  });

  socket.on("message", (rawData, isBinary) => {
    handleClientMessage(socket, rawData, isBinary);
  });

  socket.on("close", () => {
    roomManager.removeSocket(socket);
  });

  socket.on("error", (error) => {
    console.error(
      `[ws:error] connection=${context.connectionID} ip=${context.clientIP}`,
      error.message
    );
  });
});

const heartbeatTimer = setInterval(() => {
  connectionManager.heartbeat(webSocketServer.clients);
  roomManager.cleanup();
}, config.heartbeatIntervalMs);
heartbeatTimer.unref();

server.listen(config.port, config.host, () => {
  const authMode = config.relayToken ? "token required" : "no token";
  console.log(
    `Stadium Emotion Relay listening on http://${config.host}:${config.port}`
  );
  console.log(`WebSocket path: ${config.websocketPath} (${authMode})`);
  console.log(`Health check: http://${config.host}:${config.port}/health`);
});

function shutdown(signal) {
  console.log(`Received ${signal}; closing relay.`);
  clearInterval(heartbeatTimer);

  for (const socket of webSocketServer.clients) {
    socket.close(1001, "Server shutting down");
  }

  webSocketServer.close(() => {
    server.close(() => process.exit(0));
  });

  setTimeout(() => process.exit(1), 5_000).unref();
}

process.once("SIGINT", () => shutdown("SIGINT"));
process.once("SIGTERM", () => shutdown("SIGTERM"));
