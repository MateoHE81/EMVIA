function readInteger(name, fallback, minimum, maximum) {
  const raw = process.env[name];
  if (raw === undefined || raw.trim() === "") {
    return fallback;
  }

  const value = Number.parseInt(raw, 10);
  if (!Number.isInteger(value) || value < minimum || value > maximum) {
    throw new Error(
      `${name} must be an integer between ${minimum} and ${maximum}.`
    );
  }

  return value;
}

function readPath(name, fallback) {
  const value = process.env[name]?.trim() || fallback;
  if (!value.startsWith("/")) {
    throw new Error(`${name} must begin with '/'.`);
  }
  return value;
}

export const config = Object.freeze({
  host: process.env.HOST?.trim() || "0.0.0.0",
  port: readInteger("PORT", 8080, 1, 65535),
  websocketPath: readPath("WS_PATH", "/ws"),
  relayToken: process.env.RELAY_TOKEN?.trim() || null,

  protocolVersion: 1,
  maxPayloadBytes: readInteger("MAX_PAYLOAD_BYTES", 16_384, 512, 1_048_576),
  maxMessagesPerSecond: readInteger("MAX_MESSAGES_PER_SECOND", 50, 1, 1_000),
  maxHomeClientsPerMatch: readInteger("MAX_HOME_CLIENTS_PER_MATCH", 500, 1, 100_000),

  heartbeatIntervalMs: readInteger("HEARTBEAT_INTERVAL_MS", 30_000, 5_000, 300_000),
  cacheWindowMs: readInteger("CACHE_WINDOW_MS", 5_000, 0, 60_000),
  maxCachedMessagesPerMatch: readInteger(
    "MAX_CACHED_MESSAGES_PER_MATCH",
    100,
    0,
    10_000
  )
});
