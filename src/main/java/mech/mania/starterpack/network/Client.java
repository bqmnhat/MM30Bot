package mech.mania.starterpack.network;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.UnknownHostException;

public class Client {
    private static final int INITIAL_TIMEOUT = 15;
    private static final int SERVER_TURN_TIMEOUT = 30;

    private final int portNumber;
    private Socket socket;
    private BufferedReader socketReader;
    private PrintWriter socketWriter;
    private boolean connected;

    public Client(int portNumber) {
        this.portNumber = portNumber;
        this.socket = null;
        this.socketReader = null;
        this.socketWriter = null;
        this.connected = false;
    }

    public void connect() {
        long startTime = System.currentTimeMillis();
        while (!connected) {
            if (System.currentTimeMillis() - startTime > INITIAL_TIMEOUT * 1000) {
                throw new RuntimeException("Timeout when trying to connect to engine at " + portNumber);
            }

            try {
                socket = new Socket("localhost", portNumber);
                socketReader = new BufferedReader(new InputStreamReader(socket.getInputStream()));
                socketWriter = new PrintWriter(socket.getOutputStream(), true);
                connected = true;
            } catch (UnknownHostException e) {
                e.printStackTrace();
            } catch (IOException e) {
                try {
                    Thread.sleep(1000); // Sleep for 1 second before retrying
                } catch (InterruptedException e1) {
                    e1.printStackTrace();
                }
            }
        }
    }

    public String read() throws IOException {
        socket.setSoTimeout(SERVER_TURN_TIMEOUT * 1000);
        String data = socketReader.readLine();
        return data;
    }

    public void write(String message) {
        socketWriter.println(message);
    }

    public void disconnect() {
        try {
            if (socketReader != null) {
                socketReader.close();
            }
            if (socketWriter != null) {
                socketWriter.close();
            }
            if (socket != null) {
                socket.close();
            }
        } catch (IOException e) {
            e.printStackTrace();
        } finally {
            connected = false;
        }
    }
}
