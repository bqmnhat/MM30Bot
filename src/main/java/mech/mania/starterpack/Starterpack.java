package mech.mania.starterpack;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import mech.mania.starterpack.game.Plane;
import mech.mania.starterpack.game.PlaneStats;
import mech.mania.starterpack.game.PlaneType;
import mech.mania.starterpack.network.Client;
import mech.mania.starterpack.network.Phase;
import mech.mania.starterpack.network.ReceivedMessage;
import mech.mania.starterpack.strategy.Strategy;

import java.io.*;
import java.util.*;

public class Starterpack {
    private static final String rawDebugEnv = System.getenv("DEBUG");
    private static final boolean DEBUG = rawDebugEnv != null && (rawDebugEnv.equals("1") || rawDebugEnv.equalsIgnoreCase("true"));

    private static final String MAIN_USAGE = """
            Usage: java -jar starterpack.jar [command]

            Command can be one of:
            run - runs your bot against an opponent
            serve - serves your bot on a port""";
    private static final String RUN_USAGE = """
            Usage: java -jar starterpack.jar run [opponent]

            Opponent can be one of: self, computerTeam0, computerTeam1""";
    private static final String SERVE_USAGE = """
            Usage: java -jar starterpack.jar serve [port]

            Port is the port to serve on""";

    private enum RunOpponent {
        SELF("self"),
        COMPUTER_TEAM_0("computerTeam0"),
        COMPUTER_TEAM_1("computerTeam1");

        private final String value;

        RunOpponent(String value) {
            this.value = value;
        }

        public String getValue() {
            return value;
        }
    }

    private static void run(RunOpponent opponent) {
        String output = "";
        ProcessBuilder runPython3 = new ProcessBuilder("python3", "run.py", opponent.getValue());
        ProcessBuilder runPython = new ProcessBuilder("python", "run.py", opponent.getValue());

        // Try python3 first
        try {
            runPython3.redirectOutput(ProcessBuilder.Redirect.INHERIT);
            runPython3.redirectError(ProcessBuilder.Redirect.INHERIT);
            Process processPython3 = runPython3.start();
            int exitCodePython3 = processPython3.waitFor();

            System.exit(exitCodePython3);  // python3 succeeded, exit
        } catch (IOException | InterruptedException e) {
            output += "Error running python3: " + e.getMessage() + "\n";
        }

        // If python3 failed, try python
        try {
            runPython.redirectOutput(ProcessBuilder.Redirect.INHERIT);
            runPython.redirectError(ProcessBuilder.Redirect.INHERIT);
            Process processPython = runPython.start();
            int exitCodePython = processPython.waitFor();

            System.exit(exitCodePython);  // python succeeded, exit
        } catch (IOException | InterruptedException e) {
            output += "Error running python: " + e.getMessage() + "\n";
        }

        // If both failed, exit with non-zero code
        System.err.println("Failed to find python or python3:\n" + output);
        System.exit(1);
    }

    private static void serve(int port) throws IOException {
        System.out.printf("Connecting to server on port %d...\n", port);

        Client client = new Client(port);

        client.connect();

        System.out.println("Connected to server on port " + port);

        ObjectMapper objectMapper = new ObjectMapper();

        Strategy strategy = null;

        while (true) {
            String rawReceived = client.read();

            if (rawReceived != null && !rawReceived.isEmpty()) {
                try {
                    ReceivedMessage receivedMessage = objectMapper.readValue(rawReceived, ReceivedMessage.class);
                    Phase phase = receivedMessage.phase();
                    JsonNode data = receivedMessage.data();

                    if (phase != Phase.FINISH) {
                        if (DEBUG) {
                            System.out.printf("Getting your bot's response to %s phase...\n", phase);
                        }
                    }

                    if (phase == Phase.HELLO_WORLD) {
                        String team = String.valueOf(data.get("team"));

                        strategy = new Strategy(team);

                        String response = "{\"good\": true}";
                        client.write(response);
                    } else if (phase == Phase.PLANE_SELECT) {
                        Map<PlaneType, Integer> output = strategy.selectPlanes();

                        if (output == null) {
                            throw new RuntimeException("Your selectPlanes strategy returned nothing (null)!");
                        }

                        String response = objectMapper.writeValueAsString(output);
                        client.write(response);
                    } else if (phase == Phase.STEER_INPUT) {
                        Map<String, Plane> planes = objectMapper.readValue(
                                objectMapper.treeAsTokens(data),
                                new TypeReference<>() {}
                        );

                        Map<String, Double> output = strategy.steerInput(planes);
                        if (output == null) {
                            throw new RuntimeException("Your steerInput strategy returned nothing (null)!");
                        }

                        String response = objectMapper.writeValueAsString(output);
                        client.write(response);
                    }else if (phase == Phase.FINISH) {
                        System.out.println(data.asText());
                        break;
                    } else {
                        throw new RuntimeException("Unknown phase type " + phase);
                    }

                    if (DEBUG) {
                        System.out.printf("Sent response to %s phase to server!\n", phase);
                    }
                } catch (Exception e) {
                    System.err.println("Something went wrong running your bot: " + e.getMessage());
                    e.printStackTrace(System.err);
                    client.write("null");
                }
            }
        }
    }

    public static void main(String[] args) throws IOException, InterruptedException {
        if (args.length < 1) {
            System.err.println(MAIN_USAGE);
            System.exit(1);
        }

        String command = args[0];

        if (command.equals("run")) {
            if (args.length != 2) {
                System.err.println(RUN_USAGE);
                System.exit(1);
            }

            String sOpponent = args[1];

            RunOpponent opponent = null;

            for (RunOpponent possible : RunOpponent.values()) {
                if (possible.getValue().equals(sOpponent)) {
                    opponent = possible;
                    break;
                }
            }

            if (opponent == null) {
                System.err.printf("Invalid opponent %s\n", opponent);
                System.err.println(RUN_USAGE);
                System.exit(1);
            }

            run(opponent);
        } else if (command.equals("serve")) {
            if (args.length != 2) {
                System.err.println(SERVE_USAGE);
                System.exit(1);
            }

            Integer port = null;

            try {
                port = Integer.decode(args[1]);
            } catch (Exception e) {
                // Do nothing
            }

            if (port == null) {
                System.err.printf("Invalid port %d\n", port);
                System.err.println(SERVE_USAGE);
                System.exit(1);
            }

            serve(port);
        } else {
            System.err.printf("Unknown command %s\n", command);
            System.err.println(MAIN_USAGE);
            System.exit(1);
        }
    }
}
