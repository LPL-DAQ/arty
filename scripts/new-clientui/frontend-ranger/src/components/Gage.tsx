import React from 'react';
import { useBackend } from '../context/BackendContext';

interface GageProps {
    name: string;
    unit?: string;
    x?: number;
    y?: number;
}

export const Gage: React.FC<GageProps> = ({ name, unit = "PSI", x = 0, y = 0 }) => {
    const { gauges } = useBackend();
    const value = gauges[name]?.toFixed(1) ?? "N/A";

    return (
        <div
            style={{
                position: 'absolute',
                left: x,
                top: y,
                width: 180,
                height: 31,
                display: 'flex',
                flexDirection: 'row',
                alignItems: 'center',
                padding: '0 5px',
                fontFamily: 'Arial, sans-serif',
                fontSize: 17,
                boxSizing: 'border-box',
            }}
        >
            <span style={{ color: '#ffffff', whiteSpace: 'nowrap' }}>{name}:</span>
            <span style={{
                color: '#2ad12f',
                width: 74,
                textAlign: 'right',
                paddingRight: 6,
                marginLeft: 'auto',
            }}>{value}</span>
            <span style={{ color: '#69eef0' }}>{unit}</span>
        </div>
    );
};
