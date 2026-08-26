import crypto from "node:crypto";

function extractClientIP(request) {
  const forwarded = request.headers["x-forwarded-for"];
  if (typeof forwarded === "string" && forwarded.length > 0) {
    return forwarded.split(",")[0].trim();
  }
  return request.socket.remoteAddress || "unknown";
}

export class ConnectionManager {
  constructor({ maxMessagesPerSecond }) {
    this.maxMessagesPerSecond = maxMessagesPerSecond;
  }

  initialize(socket, request) {
    socket.relayContext = {
      connectionID: crypto.randomUUID(),
      clientIP: extractClientIP(request),
      connectedUtcMs: Date.now(),
      role: "unknown",
      matchID: null,
      deviceID: null,
      isAlive: true,
      rateWindowStartedUtcMs: Date.now(),
      messagesInWindow: 0,
      protocolErrors: 0
    };

    socket.on("pong", () => {
      if (socket.relayContext) {
        socket.relayContext.isAlive = true;
      }
    });

    return socket.relayContext;
  }

  allowMessage(socket, now = Date.now()) {
    const context = socket.relayContext;
    if (!context) {
      return false;
    }

    if (now - context.rateWindowStartedUtcMs >= 1_000) {
      context.rateWindowStartedUtcMs = now;
      context.messagesInWindow = 0;
    }

    context.messagesInWindow += 1;
    return context.messagesInWindow <= this.maxMessagesPerSecond;
  }

  recordProtocolError(socket) {
    if (!socket.relayContext) {
      return 0;
    }

    socket.relayContext.protocolErrors += 1;
    return socket.relayContext.protocolErrors;
  }

  resetProtocolErrors(socket) {
    if (socket.relayContext) {
      socket.relayContext.protocolErrors = 0;
    }
  }

  heartbeat(sockets) {
    for (const socket of sockets) {
      const context = socket.relayContext;
      if (!context) {
        continue;
      }

      if (context.isAlive === false) {
        socket.terminate();
        continue;
      }

      context.isAlive = false;
      socket.ping();
    }
  }
}

export function requestHasValidToken(request, expectedToken) {
  if (!expectedToken) {
    return true;
  }

  const requestURL = new URL(request.url, "http://relay.local");
  const queryToken = requestURL.searchParams.get("token");

  const authorization = request.headers.authorization;
  const bearerToken =
    typeof authorization === "string" && authorization.startsWith("Bearer ")
      ? authorization.slice("Bearer ".length).trim()
      : null;

  const provided = queryToken || bearerToken;
  if (!provided) {
    return false;
  }

  const expectedBuffer = Buffer.from(expectedToken);
  const providedBuffer = Buffer.from(provided);

  return (
    expectedBuffer.length === providedBuffer.length &&
    crypto.timingSafeEqual(expectedBuffer, providedBuffer)
  );
}
