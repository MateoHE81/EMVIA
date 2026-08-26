const WEB_SOCKET_OPEN = 1;

class MatchRoom {
  constructor(matchID, options) {
    this.matchID = matchID;
    this.options = options;
    this.homeClients = new Set();
    this.stadiumClients = new Set();
    this.cache = [];
    this.recentKeys = new Map();
    this.lastActivityUtcMs = Date.now();
  }

  touch(now = Date.now()) {
    this.lastActivityUtcMs = now;
  }

  prune(now = Date.now()) {
    const oldestAllowed = now - this.options.cacheWindowMs;

    if (this.options.cacheWindowMs <= 0) {
      this.cache.length = 0;
      this.recentKeys.clear();
      return;
    }

    while (
      this.cache.length > 0 &&
      this.cache[0].relayReceivedUtcMs < oldestAllowed
    ) {
      this.cache.shift();
    }

    for (const [key, timestamp] of this.recentKeys) {
      if (timestamp < oldestAllowed) {
        this.recentKeys.delete(key);
      }
    }
  }

  addHomeClient(socket) {
    if (
      !this.homeClients.has(socket) &&
      this.homeClients.size >= this.options.maxHomeClientsPerMatch
    ) {
      throw new Error(`Match ${this.matchID} has reached its Home App limit.`);
    }

    this.homeClients.add(socket);
    this.touch();
  }

  addStadiumClient(socket) {
    this.stadiumClients.add(socket);
    this.touch();
  }

  remove(socket) {
    this.homeClients.delete(socket);
    this.stadiumClients.delete(socket);
    this.touch();
  }

  publish(message, sender, now = Date.now()) {
    this.prune(now);
    this.addStadiumClient(sender);

    const deduplicationKey = [
      message.deviceID,
      message.sequence,
      message.eventUtcMs
    ].join(":");

    if (this.recentKeys.has(deduplicationKey)) {
      return { duplicate: true, forwarded: 0 };
    }

    this.recentKeys.set(deduplicationKey, now);

    // The original validated message is forwarded. eventUtcMs and sequence are
    // never replaced by server arrival time or regenerated server values.
    const payload = JSON.stringify(message);
    let forwarded = 0;

    for (const client of this.homeClients) {
      if (client.readyState === WEB_SOCKET_OPEN) {
        client.send(payload);
        forwarded += 1;
      }
    }

    if (this.options.maxCachedMessagesPerMatch > 0) {
      this.cache.push({
        eventUtcMs: message.eventUtcMs,
        relayReceivedUtcMs: now,
        payload
      });

      while (this.cache.length > this.options.maxCachedMessagesPerMatch) {
        this.cache.shift();
      }
    }

    this.touch(now);
    return { duplicate: false, forwarded };
  }

  replay(socket, resumeAfterEventUtcMs) {
    if (resumeAfterEventUtcMs === null) {
      return 0;
    }

    this.prune();
    let replayed = 0;

    for (const item of this.cache) {
      if (
        item.eventUtcMs > resumeAfterEventUtcMs &&
        socket.readyState === WEB_SOCKET_OPEN
      ) {
        socket.send(item.payload);
        replayed += 1;
      }
    }

    return replayed;
  }

  get isUnused() {
    return this.homeClients.size === 0 && this.stadiumClients.size === 0;
  }

  summary() {
    return {
      matchID: this.matchID,
      homeClients: this.homeClients.size,
      stadiumClients: this.stadiumClients.size,
      cachedMessages: this.cache.length,
      lastActivityUtcMs: this.lastActivityUtcMs
    };
  }
}

export class MatchRoomManager {
  constructor(options) {
    this.options = options;
    this.rooms = new Map();
  }

  getOrCreate(matchID) {
    let room = this.rooms.get(matchID);
    if (!room) {
      room = new MatchRoom(matchID, this.options);
      this.rooms.set(matchID, room);
    }
    return room;
  }

  subscribeHome(socket, subscription) {
    this.removeSocket(socket);

    const room = this.getOrCreate(subscription.matchID);
    room.addHomeClient(socket);

    socket.relayContext.role = "home";
    socket.relayContext.matchID = subscription.matchID;

    const replayed = room.replay(
      socket,
      subscription.resumeAfterEventUtcMs
    );

    return {
      matchID: subscription.matchID,
      homeClients: room.homeClients.size,
      replayed
    };
  }

  publishEmotion(socket, message) {
    const previousMatchID = socket.relayContext.matchID;
    if (previousMatchID && previousMatchID !== message.matchID) {
      this.removeSocket(socket);
    }

    const room = this.getOrCreate(message.matchID);
    socket.relayContext.role = "stadium";
    socket.relayContext.matchID = message.matchID;
    socket.relayContext.deviceID = message.deviceID;

    return room.publish(message, socket);
  }

  removeSocket(socket) {
    const matchID = socket.relayContext?.matchID;
    if (!matchID) {
      return;
    }

    const room = this.rooms.get(matchID);
    room?.remove(socket);

    if (socket.relayContext) {
      socket.relayContext.role = "unknown";
      socket.relayContext.matchID = null;
      socket.relayContext.deviceID = null;
    }
  }

  cleanup(now = Date.now()) {
    const deleteBefore = now - Math.max(this.options.cacheWindowMs, 1_000);

    for (const [matchID, room] of this.rooms) {
      room.prune(now);
      if (room.isUnused && room.lastActivityUtcMs < deleteBefore) {
        this.rooms.delete(matchID);
      }
    }
  }

  summaries() {
    return Array.from(this.rooms.values(), (room) => room.summary());
  }

  get totalHomeClients() {
    let total = 0;
    for (const room of this.rooms.values()) {
      total += room.homeClients.size;
    }
    return total;
  }

  get totalStadiumClients() {
    let total = 0;
    for (const room of this.rooms.values()) {
      total += room.stadiumClients.size;
    }
    return total;
  }
}
