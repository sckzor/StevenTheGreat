package frc.robot;

import java.net.DatagramSocket;
import java.net.DatagramPacket;
import java.net.InetSocketAddress;
import java.util.ArrayList;

public class CustomController extends Thread {
    public static DatagramSocket saket;
    public static DatagramPacket paket;
    public static ArrayList<Integer> keycodeQueue;

    public static void InitControlSocket() {
        keycodeQueue = new ArrayList<Integer>();
        if (saket == null) {
            try {
                saket = new DatagramSocket(null);
            } catch (Exception e) {
                System.err.println(e);
            }
        }
        
        try {
            saket.bind(new InetSocketAddress(5800));
        } catch (Exception e) {
            System.err.println(e);
        }

        if (paket == null) {
            paket = new DatagramPacket(new byte[1500], 1500);
        }
    }

    public static void UpdateControlSocket()
    {
        try {
            saket.receive(paket);
        } catch (Exception e) {
            System.err.println(e);
        }

        int keycode = Integer.valueOf(new String(paket.getData(), paket.getOffset(), paket.getLength()));
        keycodeQueue.add(keycode);
    }

    @Override
    public void run()
    {
        InitControlSocket();
        while(true) {
            UpdateControlSocket();
        }
    }
    
    public Integer PickKeyCode() {
        if( keycodeQueue.size() != 0) {
            return keycodeQueue.remove(keycodeQueue.size() - 1);
        }
        return -1;
    }
}
