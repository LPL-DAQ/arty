import React from 'react';
import { useBackend } from '../context/BackendContext';

interface ValveToggleProps {
    name: string;
}

/**
 * Right-panel toggle matching QML ValveToggle.qml:
 * Left half: grey button with 3 stripes (gold when armed) + name label
 * Right half: OPENED/CLOSED + PWR ON/OFF stacked indicators
 */
export const ValveToggle: React.FC<ValveToggleProps> = ({ name }) => {
    const { valves, armed, armValve, toggleValve } = useBackend();
    const backendKey = name.replace(/-/g, "");
    const isOpen = valves[backendKey] === "OPEN";
    const isArmed = armed[backendKey] ?? false;

    const stripColor = isArmed ? '#d9a637' : '#dbdbdb';

    return (
        <div style={{
            width: 208,
            height: 65,
            display: 'flex',
            flexDirection: 'row',
            border: '1px solid #7f5d5d',
            boxSizing: 'border-box',
        }}>
            {/* Left: Selection button */}
            <div
                style={{
                    width: 101,
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
                <div style={{ display: 'flex', flexDirection: 'column', gap: 3, marginBottom: 4, width: '70%' }}>
                    <div style={{ height: 7, backgroundColor: stripColor }} />
                    <div style={{ height: 7, backgroundColor: stripColor }} />
                    <div style={{ height: 7, backgroundColor: stripColor }} />
                </div>
                <span style={{
                    color: '#ffffff', fontSize: 18, fontFamily: 'Arial',
                    textAlign: 'center',
                }}>{name}</span>
            </div>

            {/* Right: Status indicators */}
            <div style={{ width: 101, height: '100%', display: 'flex', flexDirection: 'column' }}>
                {/* Top: OPENED/CLOSED */}
                <div
                    onClick={() => toggleValve(backendKey)}
                    style={{
                        flex: 1,
                        backgroundColor: '#000000',
                        border: '1px solid #bebebe',
                        display: 'flex',
                        alignItems: 'center',
                        justifyContent: 'center',
                        fontSize: 18,
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
                    fontSize: 18,
                    fontFamily: 'Arial',
                    color: isArmed ? '#69eef0' : '#7f7f7f',
                    boxSizing: 'border-box',
                }}>
                    {isArmed ? 'PWR ON' : 'PWR OFF'}
                </div>
            </div>
        </div>
    );
};
