import React from 'react';
import { useBackend } from '../context/BackendContext';

interface ValveToggleProps {
    name: string;
    /** Overall card width (px). Default 208. */
    width?: number;
    /** Overall card height (px). Default 65. */
    height?: number;
    /** Font size override for valve ID + state labels (px). Default scales from height. */
    fontSize?: number;
}

/**
 * Right-panel toggle matching QML ValveToggle.qml:
 * Left half: grey button with 3 stripes (gold when armed) + name label
 * Right half: OPENED/CLOSED + PWR ON/OFF stacked indicators
 *
 * Sizes are configurable so the dashboard can render compact variants
 * matching the Atlas mission-control reference.
 */
export const ValveToggle: React.FC<ValveToggleProps> = ({
    name,
    width = 208,
    height = 65,
    fontSize,
}) => {
    const { valves, armed, powered, armValve } = useBackend();
    const backendKey = name.replace(/-/g, "");
    const isOpen = valves[backendKey] === "OPEN";
    const isArmed = armed[backendKey] ?? false;
    const isPowered = powered[backendKey] ?? false;

    const stripColor = isArmed ? '#d9a637' : '#dbdbdb';

    // Scale font sizes with height so compact cards stay readable.
    const fs = fontSize ?? Math.max(12, Math.round(height * 0.26));
    const stripeH = Math.max(4, Math.round(height * 0.1));
    const stripeGap = Math.max(2, Math.round(height * 0.045));
    const halfW = Math.floor(width / 2);

    return (
        <div style={{
            width,
            height,
            display: 'flex',
            flexDirection: 'row',
            border: '1px solid #7f5d5d',
            boxSizing: 'border-box',
        }}>
            {/* Left: Selection button */}
            <div
                style={{
                    width: halfW,
                    height: '100%',
                    backgroundColor: '#555555',
                    border: '1px solid #ffffff',
                    cursor: 'pointer',
                    display: 'flex',
                    flexDirection: 'column',
                    alignItems: 'center',
                    justifyContent: 'center',
                    position: 'relative',
                    boxSizing: 'border-box',
                }}
                onClick={() => armValve(backendKey)}
            >
                {/* 3 stripes */}
                <div style={{ display: 'flex', flexDirection: 'column', gap: stripeGap, marginBottom: stripeGap + 1, width: '68%' }}>
                    <div style={{ height: stripeH, backgroundColor: stripColor }} />
                    <div style={{ height: stripeH, backgroundColor: stripColor }} />
                    <div style={{ height: stripeH, backgroundColor: stripColor }} />
                </div>
                <span style={{
                    color: '#ffffff', fontSize: fs, fontFamily: 'Arial',
                    textAlign: 'center', letterSpacing: 0.3,
                }}>{name}</span>
            </div>

            {/* Right: Status indicators */}
            <div style={{ width: width - halfW, height: '100%', display: 'flex', flexDirection: 'column' }}>
                {/* Top: OPENED/CLOSED */}
                <div
                    style={{
                        flex: 1,
                        backgroundColor: '#000000',
                        border: '1px solid #bebebe',
                        display: 'flex',
                        alignItems: 'center',
                        justifyContent: 'center',
                        fontSize: fs,
                        fontFamily: 'Arial',
                        color: isOpen ? '#2ad12f' : '#7f7f7f',
                        boxSizing: 'border-box',
                        cursor: 'pointer',
                    }}>
                    {isOpen ? 'OPENED' : 'CLOSED'}
                </div>
                {/* Bottom: PWR ON/OFF */}
                <div style={{
                    flex: 1,
                    backgroundColor: '#000000',
                    border: '1px solid #bebebe',
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'center',
                    fontSize: fs,
                    fontFamily: 'Arial',
                    color: isPowered ? '#69eef0' : '#7f7f7f',
                    boxSizing: 'border-box',
                }}>
                    {isPowered ? 'PWR ON' : 'PWR OFF'}
                </div>
            </div>
        </div>
    );
};
