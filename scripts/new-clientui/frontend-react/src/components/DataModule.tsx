import React from 'react';
import { useBackend } from '../context/BackendContext';

interface DataModuleProps {
    name: string;
    label?: string;
    unit?: string;
    max?: number; // For progress bar
    className?: string;
}

export const DataModule: React.FC<DataModuleProps> = ({ name, label, unit = "PSI", max = 1000, className }) => {
    const { gauges } = useBackend();
    const rawValue = gauges[name] || 0;
    const value = rawValue.toFixed(1);
    const percent = Math.min((rawValue / max) * 100, 100);

    // Dynamic Color
    let colorClass = "text-[var(--color-primary)]";
    if (percent > 90) {
        colorClass = "text-[var(--color-danger)]";
    } else if (percent > 75) {
        colorClass = "text-[var(--color-warning)]";
    }

    return (
        <div className={`p-4 bg-gray-800 rounded border border-gray-700 ${className} flex flex-col justify-center min-h-[80px]`}>
            <div className="flex justify-between items-baseline mb-1">
                <span className="text-xs font-bold uppercase text-gray-400">{label || name}</span>
                <div className="flex items-baseline">
                    <span className={`font-mono text-3xl font-bold ${colorClass}`}>
                        {value}
                    </span>
                    <span className="text-xs text-gray-500 ml-1 font-bold">{unit}</span>
                </div>
            </div>

            {/* Simple sparkline or subtext if needed, keeping empty for now to match 'minimal' */}
        </div>
    );
};
