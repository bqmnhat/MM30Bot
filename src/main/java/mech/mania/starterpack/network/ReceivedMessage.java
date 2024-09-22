package mech.mania.starterpack.network;

import com.fasterxml.jackson.databind.JsonNode;

public record ReceivedMessage(Phase phase, JsonNode data) {}
