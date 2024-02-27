# Health Packet Package

## Purpose
Provides a (service client) node that will send requests to the Basestation Health Service. If there is an error when attempting to send a request a message should be sent to the main node that results in a hard stop of all movement of the rover.

## Service
The service should be hosted by the basestation node. It is expected to be under `/astra/core/health`.

## Shutdown Topic
This node will host a topic for shutdown messages at `/astra/hard_stop`. It is expected for all relay nodes to subscribe to this topic and force a complete stop in the event that a message is sent instructing shutdown.