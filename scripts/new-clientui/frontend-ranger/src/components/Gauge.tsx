import React from 'react';
import { useBackend } from '../context/BackendContext';

interface GaugeProps {
    name: string; // Sensor ID, e.g., "PT401"
    label?: string; // Display label if different from ID
    unit?: string;
    className?: string;
    style?: React.CSSProperties;
}

export const Gauge: React.FC<GaugeProps> = ({ name, label, unit = "PSI", className, style }) => {
    const { gauges } = useBackend();
    const value = gauges[name]?.toFixed(1) || "---";

    return (
        <div
            className={`flex items-center justify-between px-3 py-1 bg-black/80 backdrop-blur-sm border border-white/30 rounded shadow-[0_4px_6px_rgba(0,0,0,0.5)] bg-gradient-to-b from-white/10 to-transparent ${className}`}
            style={{ width: '180px', height: '36px', ...style }}
        >
            <span className="font-bold text-gray-200 drop-shadow-[0_1px_1px_rgba(0,0,0,0.8)]">{label || name}</span>
            <div className="bg-black/90 border border-white/10 rounded px-2 py-0.5 shadow-[inset_0_2px_4px_rgba(0,0,0,0.8)] min-w-[80px] text-right">
                <span className="text-green-400 font-mono text-lg drop-shadow-[0_0_2px_rgba(74,222,128,0.5)]">
                    {value} <span className="text-xs text-gray-500">{unit}</span>
                </span>
            </div>
        </div>
    );
};
