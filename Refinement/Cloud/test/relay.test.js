import test from "node:test";
import assert from "node:assert/strict";
import {
  RelayProtocolError,
  validateEmotionMessage,
  validateSubscribeMessage
} from "../emotionMessage.js";
import { MatchRoomManager } from "../matchRoomManager.js";

function makeEmotion(overrides = {}) {
  return {
    type: "emotion",
    protocolVersion: 1,
    matchID: "demo-match",
    source: "stadium-ios",
    deviceID: 1,
    sequence: 42,
    eventUtcMs: 1_785_691_220_000,
    receivedUtcMs: 1_785_691_220_012,
    deviceTimestampMs: 123_456,
    ces: 0.82,
    spike: 0.74,
    audio: 0.91,
    heart: 0.52,
    motion: 0.43,
    bpm: 104,
    fusionQuality: 1,
    statusFlags: 63,
    ...overrides
  };
}

function fakeSocket() {
  return {
    readyState: 1,
    sent: [],
    relayContext: {
      role: "unknown",
      matchID: null,
      deviceID: null
    },
    send(payload) {
      this.sent.push(JSON.parse(payload));
    }
  };
}

test("validates the Stadium App emotion JSON without changing timing fields", () => {
  const input = makeEmotion();
  const result = validateEmotionMessage(input, 1);

  assert.equal(result.eventUtcMs, input.eventUtcMs);
  assert.equal(result.sequence, input.sequence);
  assert.equal(result.ces, input.ces);
  assert.equal(result.spike, input.spike);
});

test("rejects normalized scores outside 0...1", () => {
  assert.throws(
    () => validateEmotionMessage(makeEmotion({ ces: 1.2 }), 1),
    RelayProtocolError
  );
});

test("validates Home App subscriptions", () => {
  const subscription = validateSubscribeMessage({
    type: "subscribe",
    protocolVersion: 1,
    role: "home",
    matchID: "demo-match"
  });

  assert.equal(subscription.matchID, "demo-match");
  assert.equal(subscription.resumeAfterEventUtcMs, null);
});

test("forwards emotion messages only to Home Apps in the same match", () => {
  const rooms = new MatchRoomManager({
    cacheWindowMs: 5_000,
    maxCachedMessagesPerMatch: 100,
    maxHomeClientsPerMatch: 10
  });

  const stadium = fakeSocket();
  const matchingHome = fakeSocket();
  const otherHome = fakeSocket();

  rooms.subscribeHome(matchingHome, {
    matchID: "demo-match",
    resumeAfterEventUtcMs: null
  });
  rooms.subscribeHome(otherHome, {
    matchID: "other-match",
    resumeAfterEventUtcMs: null
  });

  rooms.publishEmotion(stadium, validateEmotionMessage(makeEmotion(), 1));

  assert.equal(matchingHome.sent.length, 1);
  assert.equal(matchingHome.sent[0].sequence, 42);
  assert.equal(otherHome.sent.length, 0);
});

test("replays only explicitly requested recent packets", () => {
  const rooms = new MatchRoomManager({
    cacheWindowMs: 5_000,
    maxCachedMessagesPerMatch: 100,
    maxHomeClientsPerMatch: 10
  });

  const stadium = fakeSocket();
  rooms.publishEmotion(
    stadium,
    validateEmotionMessage(makeEmotion({ sequence: 1, eventUtcMs: 1000 }), 1)
  );
  rooms.publishEmotion(
    stadium,
    validateEmotionMessage(makeEmotion({ sequence: 2, eventUtcMs: 2000 }), 1)
  );

  const home = fakeSocket();
  const result = rooms.subscribeHome(home, {
    matchID: "demo-match",
    resumeAfterEventUtcMs: 1000
  });

  assert.equal(result.replayed, 1);
  assert.equal(home.sent[0].sequence, 2);
});
