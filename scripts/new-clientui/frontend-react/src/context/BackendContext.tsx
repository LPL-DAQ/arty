import React, { createContext, useContext, useState, useEffect, useCallback } from 'react';
import type { ReactNode } from 'react';

interface BackendContextType {
    gauges: Record<string, number>;
    valves: Record<string, "OPEN" | "CLOSE">;
    armed: Record<string, boolean>;
    timer: number;
    serverStatus: boolean;
    abortStatus: boolean;
    toggleValve: (name: string) => void;
    armValve: (name: string) => void;
    actuate: () => void;
    abort: () => void;
    reset: () => void;
    initiateSequence: (name: string) => void;
    startCountdown: (name: string, sequenceStartTime?: number) => void;
}

const BackendContext = createContext<BackendContextType | undefined>(undefined);



export const BackendProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
    const [gauges, setGauges] = useState<Record<string, number>>({});
    const [valves, setValves] = useState<Record<string, "OPEN" | "CLOSE">>({});
    const [armed, setArmed] = useState<Record<string, boolean>>({});
    const [serverStatus, setServerStatus] = useState(false);
    const [abortStatus, setAbortStatus] = useState(false);
    const [timer, setTimer] = useState(15.0);
    const wsRef = React.useRef<WebSocket | null>(null);
    const countdownIntervalRef = React.useRef<ReturnType<typeof setInterval> | null>(null);

    useEffect(() => {
        // Simple WebSocket implementation for real-time streaming
        const connectWs = () => {
            const ws = new WebSocket('ws://localhost:8000/ws');
            wsRef.current = ws;

            ws.onopen = () => setServerStatus(true);
            ws.onclose = () => {
                setServerStatus(false);
                setTimeout(connectWs, 2000); // Auto-reconnect
            };
            ws.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);
                    if (data.gauges) setGauges(prev => ({ ...prev, ...data.gauges }));
                    if (data.valves) setValves(prev => ({ ...prev, ...data.valves }));
                    if (data.armed) setArmed(prev => ({ ...prev, ...data.armed }));
                } catch (e) {
                    console.error("Failed to parse WS message", e);
                }
            };
        };

        connectWs();

        return () => {
            wsRef.current?.close();
        };
    }, []);

    const sendMessage = useCallback((msg: any) => {
        if (wsRef.current?.readyState === WebSocket.OPEN) {
            wsRef.current.send(JSON.stringify(msg));
        } else {
            console.warn("WebSocket not connected. Cannot send: ", msg);
        }
    }, []);

    const toggleValve = useCallback((name: string) => {
        setValves(prev => ({
            ...prev,
            [name]: prev[name] === "OPEN" ? "CLOSE" : "OPEN"
        }));
        sendMessage({ action: "TOGGLE_VALVE", name });
    }, [sendMessage]);

    const armValve = useCallback((name: string) => {
        // Optimistic UI update: Exclusive selection
        setArmed(prev => {
            const isCurrentlyArmed = prev[name] ?? false;
            const newArmed: Record<string, boolean> = {};
            // Set all to false, toggle only the target if it wasn't already armed
            Object.keys(prev).forEach(k => {
                newArmed[k] = (k === name) ? !isCurrentlyArmed : false;
            });
            return newArmed;
        });
        sendMessage({ action: "ARM_VALVE", name });
    }, [sendMessage]);

    const actuate = useCallback(() => {
        sendMessage({ action: "ACTUATE" });
    }, [sendMessage]);

    const abort = useCallback(() => {
        if (countdownIntervalRef.current) clearInterval(countdownIntervalRef.current);
        setAbortStatus(true);
        sendMessage({ action: "ABORT" });
    }, [sendMessage]);

    const reset = useCallback(() => {
        if (countdownIntervalRef.current) clearInterval(countdownIntervalRef.current);
        setAbortStatus(false);
        setTimer(15.0);
        sendMessage({ action: "RESET" });
    }, [sendMessage]);

    const initiateSequence = useCallback((name: string) => {
        sendMessage({ action: "INITIATE_SEQUENCE", name });
    }, [sendMessage]);

    const startCountdown = useCallback((name: string, sequenceStartTime: number = 0) => {
        if (countdownIntervalRef.current) clearInterval(countdownIntervalRef.current);

        // Start 15 seconds before the sequence's start time
        let ticks = (15 + sequenceStartTime) * 10;
        setTimer(ticks / 10.0);
        let sequenceTriggered = false;

        countdownIntervalRef.current = setInterval(() => {
            ticks -= 1;
            const currentTimerValue = ticks / 10.0;
            setTimer(currentTimerValue);

            // Trigger sequence when we reach the sequence's start time (exactly 15s after counting starts)
            if (!sequenceTriggered && currentTimerValue <= sequenceStartTime + 0.01) {
                sendMessage({ action: "INITIATE_SEQUENCE", name });
                sequenceTriggered = true;
            }

            // Stop at 0.0
            if (ticks <= 0) {
                if (countdownIntervalRef.current) clearInterval(countdownIntervalRef.current);
            }
        }, 100);
    }, [sendMessage]);

    return (
        <BackendContext.Provider value={{
            gauges, valves, armed, timer, serverStatus, abortStatus,
            toggleValve, armValve, actuate, abort, reset, initiateSequence, startCountdown
        }}>
            {children}
        </BackendContext.Provider>
    );
};

export const useBackend = () => {
    const ctx = useContext(BackendContext);
    if (!ctx) throw new Error("useBackend must be used within BackendProvider");
    return ctx;
};
