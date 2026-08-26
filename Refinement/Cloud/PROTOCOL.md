# Cloud Relay Protocol

## Server to every newly connected client

```json
{
  "type": "hello",
  "protocolVersion": 1,
  "connectionID": "uuid",
  "serverUtcMs": 1785691220015,
  "websocketPath": "/ws"
}
```

## Stadium publisher

The Stadium App sends the `emotion` object defined in `README.md`. The first valid packet registers that connection as a stadium publisher. The server may reply once with:

```json
{
  "type": "publisherReady",
  "protocolVersion": 1,
  "role": "stadium",
  "matchID": "demo-match",
  "deviceID": 1,
  "serverUtcMs": 1785691220020
}
```

## Home subscriber

```json
{
  "type": "subscribe",
  "protocolVersion": 1,
  "role": "home",
  "matchID": "demo-match"
}
```

Response:

```json
{
  "type": "subscribed",
  "protocolVersion": 1,
  "role": "home",
  "matchID": "demo-match",
  "homeClients": 1,
  "replayed": 0,
  "serverUtcMs": 1785691220025
}
```

## Unsubscribe

```json
{
  "type": "unsubscribe",
  "protocolVersion": 1
}
```

## Error

```json
{
  "type": "error",
  "protocolVersion": 1,
  "code": "INVALID_FIELD",
  "message": "ces must be a finite number between 0 and 1.",
  "serverUtcMs": 1785691220030
}
```
