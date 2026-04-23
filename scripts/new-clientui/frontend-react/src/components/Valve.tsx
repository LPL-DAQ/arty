import React from 'react';
import { useBackend } from '../context/BackendContext';

interface ValveProps {
    name: string;
    x: number;
    y: number;
    width?: number;
    height?: number;
}

/**
 * Color-coded valve overlay matching QML LoxBallValve.qml:
 * - OD (Open Disarmed) = green
 * - OA (Open Armed) = orange
 * - CD (Closed Disarmed) = white/grey
 * - CA (Closed Armed) = orange
 */
export const Valve: React.FC<ValveProps> = ({ name, x, y, width = 67, height = 29 }) => {
    const { valves, armed } = useBackend();
    const backendKey = name.replace(/-/g, "");
    const isOpen = valves[backendKey] === "OPEN";
    const isArmed = armed[backendKey] ?? false;

    let color: string;
    if (isOpen && isArmed) color = '#d9a637';      // OA - orange
    else if (isOpen && !isArmed) color = '#2ad12f'; // OD - green
    else if (!isOpen && isArmed) color = '#d9a637';  // CA - orange
    else color = '#cccccc';                           // CD - white/grey

    return (
        <div
            style={{
                position: 'absolute',
                left: x,
                top: y,
                width,
                height,
                backgroundColor: color,
                borderRadius: 3,
                transition: 'background-color 0.15s ease',
                opacity: 0.85,
            }}
            title={`${name}: ${isOpen ? 'OPEN' : 'CLOSED'}${isArmed ? ' (ARMED)' : ''}`}
        />
    );
};
