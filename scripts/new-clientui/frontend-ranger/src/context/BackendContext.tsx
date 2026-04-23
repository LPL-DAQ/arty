import React, { createContext, useContext, useState, useEffect, useCallback } from 'react';
import type { ReactNode } from 'react';

interface BackendContextType {
    gauges: Record<string, number>;
    valves: Record<string, "OPEN" | "CLOSE">;
    armed: Record<string, boolean>;
    powered: Record<string, boolean>;
    sequences: { id: string; name: string; startTime: number; tMin: number; tMax: number }[];
    timer: number;
    serverStatus: boolean;
    abortStatus: boolean;
    toggleValve: (name: string) => void;
    armValve: (name: string) => void;
    actuate: () => void;
    abort: () => void;
    reset: () => void;
    initiateSequence: (name: string) => void;
    /** Mission clock: counts from tMin-15 through tMax+15; fires sequence at tMin. */
    startCountdown: (name: string, tMin?: number, tMax?: number) => void;
    stopSequence: () => void;
    refreshSequences: () => void;
}

const BackendContext = createContext<BackendContextType | undefined>(undefined);

const normalizeKey = (name: string): string => name.replace(/[^a-zA-Z0-9]/g, '').toUpperCase();

const normalizeMap = <T,>(obj: Record<string, T>): Record<string, T> =>
    Object.fromEntries(Object.entries(obj).map(([k, v]) => [normalizeKey(k), v]));


export const BackendProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
    const [gauges, setGauges] = useState<Record<string, number>>({});
    const [valves, setValves] = useState<Record<string, "OPEN" | "CLOSE">>({});
    const [armed, setArmed] = useState<Record<string, boolean>>({});
    const [powered, setPowered] = useState<Record<string, boolean>>({});
    const [sequences, setSequences] = useState<{ id: string; name: string; startTime: number; tMin: number; tMax: number }[]>([]);
    const [serverStatus, setServerStatus] = useState(false);
    const [abortStatus, setAbortStatus] = useState(false);
    const [timer, setTimer] = useState(0);
    const wsRef = React.useRef<WebSocket | null>(null);
    const countdownIntervalRef = React.useRef<ReturnType<typeof setInterval> | null>(null);

    useEffect(() => {
        // Simple WebSocket implementation for real-time streaming
        const connectWs = () => {
            const wsUrl = import.meta.env.VITE_BACKEND_WS_URL ?? 'ws://localhost:8000/ws';
            const ws = new WebSocket(wsUrl);
            wsRef.current = ws;

            ws.onopen = () => setServerStatus(true);
            ws.onclose = () => {
                setServerStatus(false);
                setTimeout(connectWs, 2000); // Auto-reconnect
            };
            ws.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);
                    if (data.gauges) setGauges(prev => ({ ...prev, ...normalizeMap(data.gauges) }));
                    if (data.valves) setValves(prev => ({ ...prev, ...normalizeMap(data.valves) }));
                    if (data.armed) setArmed(prev => ({ ...prev, ...normalizeMap(data.armed) }));
                    if (data.powered) setPowered(prev => ({ ...prev, ...normalizeMap(data.powered) }));
                    if (Array.isArray(data.sequences)) {
                        setSequences(data.sequences.map((s: {
                            id: string; name: string; startTime?: number; tMin?: number; tMax?: number;
                        }) => ({
                            id: s.id,
                            name: s.name,
                            startTime: s.startTime ?? 0,
                            tMin: s.tMin ?? -6,
                            tMax: s.tMax ?? 9,
                        })));
                    }
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
        const normalized = normalizeKey(name);
        setValves(prev => ({
            ...prev,
            [normalized]: prev[normalized] === "OPEN" ? "CLOSE" : "OPEN"
        }));
        sendMessage({ action: "TOGGLE_VALVE", name: normalized });
    }, [sendMessage]);

    const armValve = useCallback((name: string) => {
        const normalized = normalizeKey(name);
        // Optimistic single-select: only one armed at a time (matches backend).
        setArmed(prev => {
            const wasThis = prev[normalized] ?? false;
            const cleared: Record<string, boolean> = {};
            Object.keys(prev).forEach(k => { cleared[k] = false; });
            return { ...cleared, [normalized]: !wasThis };
        });
        sendMessage({ action: "ARM_VALVE", name: normalized });
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
        setTimer(0);
        sendMessage({ action: "RESET" });
    }, [sendMessage]);

    const initiateSequence = useCallback((name: string) => {
        sendMessage({ action: "INITIATE_SEQUENCE", name });
    }, [sendMessage]);

    const stopSequence = useCallback(() => {
        sendMessage({ action: "STOP_SEQUENCE" });
    }, [sendMessage]);

    const refreshSequences = useCallback(() => {
        sendMessage({ action: "GET_SEQUENCES" });
    }, [sendMessage]);

    const startCountdown = useCallback((name: string, tMin: number = -6, tMax: number = 9) => {
        if (countdownIntervalRef.current) clearInterval(countdownIntervalRef.current);
        const startDisplay = tMin - 15;
        const endDisplay = tMax + 15;
        let missionT = startDisplay;
        setTimer(Math.round(missionT * 10) / 10);
        let sequenceTriggered = false;

        countdownIntervalRef.current = setInterval(() => {
            missionT += 0.1;
            missionT = Math.round(missionT * 10) / 10;
            setTimer(missionT);

            if (!sequenceTriggered && missionT >= tMin - 0.001) {
                sendMessage({ action: "INITIATE_SEQUENCE", name });
                sequenceTriggered = true;
            }

            if (missionT >= endDisplay - 0.001) {
                if (countdownIntervalRef.current) clearInterval(countdownIntervalRef.current);
            }
        }, 100);
    }, [sendMessage]);

    return (
        <BackendContext.Provider value={{
            gauges, valves, armed, powered, sequences, timer, serverStatus, abortStatus,
            toggleValve, armValve, actuate, abort, reset, initiateSequence, startCountdown, stopSequence, refreshSequences
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
