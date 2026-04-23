import React from 'react';
import { useBackend } from '../context/BackendContext';

interface ValveStateProps {
    name: string;
    x: number;
    y: number;
}

/**
 * Label beneath each valve on the schematic.
 * QML: 85×62 transparent rect, name in white (15px), state in green/red (15px).
 */
export const ValveState: React.FC<ValveStateProps> = ({ name, x, y }) => {
    const { valves } = useBackend();
    const backendKey = name.replace(/-/g, "");
    const state = valves[backendKey];

    let stateText: string;
    let stateColor: string;
    if (state === "OPEN") { stateText = "OPENED"; stateColor = "#10ff00"; }
    else if (state === "CLOSE") { stateText = "CLOSED"; stateColor = "#ff0000"; }
    else { stateText = "N/A"; stateColor = "#7f7f7f"; }

    return (
        <div
            style={{
                position: 'absolute',
                left: x,
                top: y,
                width: 85,
                height: 62,
                textAlign: 'center',
                fontFamily: 'Arial, sans-serif',
                fontSize: 15,
            }}
        >
            <div style={{ color: '#ffffff', marginTop: 2 }}>{name}</div>
            <div style={{ color: stateColor, marginTop: 10 }}>{stateText}</div>
        </div>
    );
};
