import React from 'react';

// --- Pipes ---
export const Pipe: React.FC<{
    x: number; y: number; w: number; h: number;
    color: string;
    direction?: 'up' | 'down' | 'left' | 'right';
}> = ({ x, y, w, h, color, direction }) => {
    return (
        <div style={{
            position: 'absolute',
            left: x, top: y,
            width: w, height: h,
            backgroundColor: color,
            zIndex: 1,
        }}>
            {direction && (
                <div style={{
                    position: 'absolute',
                    ...(direction === 'right' ? { right: -6, top: '50%', transform: 'translateY(-50%)', borderTop: '6px solid transparent', borderBottom: '6px solid transparent', borderLeft: `8px solid ${color}` } : {}),
                    ...(direction === 'left' ? { left: -6, top: '50%', transform: 'translateY(-50%)', borderTop: '6px solid transparent', borderBottom: '6px solid transparent', borderRight: `8px solid ${color}` } : {}),
                    ...(direction === 'up' ? { top: -6, left: '50%', transform: 'translateX(-50%)', borderLeft: '6px solid transparent', borderRight: '6px solid transparent', borderBottom: `8px solid ${color}` } : {}),
                    ...(direction === 'down' ? { bottom: -6, left: '50%', transform: 'translateX(-50%)', borderLeft: '6px solid transparent', borderRight: '6px solid transparent', borderTop: `8px solid ${color}` } : {}),
                }} />
            )}
        </div>
    );
};

// --- Tanks ---
export const Tank: React.FC<{
    x: number; y: number; w: number; h: number;
    color: string;
    name: string;
    capacity?: string;
    meop?: string;
    vertical?: boolean;
}> = ({ x, y, w, h, color, name, capacity, meop, vertical }) => {
    return (
        <div style={{
            position: 'absolute',
            left: x, top: y,
            width: w, height: h,
            backgroundColor: color,
            borderRadius: vertical ? w / 2 : h / 2,
            border: '2px solid black',
            display: 'flex',
            flexDirection: 'column',
            alignItems: 'center',
            justifyContent: 'center',
            color: 'white',
            fontWeight: 'bold',
            fontSize: 10,
            textAlign: 'center',
            zIndex: 2,
            boxShadow: 'inset -5px -5px 15px rgba(0,0,0,0.3)',
            padding: 5,
            boxSizing: 'border-box',
        }}>
            <div style={{ fontSize: 12 }}>{name}</div>
            {capacity && <div style={{ fontSize: 9, fontWeight: 'normal', marginTop: 2 }}>{capacity}</div>}
            {meop && <div style={{ fontSize: 9, fontWeight: 'normal' }}>{meop}</div>}
        </div>
    );
};

// --- P&ID Valve Symbols (SVG) ---
export const ValveSymbol: React.FC<{
    x: number; y: number;
    type?: 'manual' | 'solenoid' | 'pneumatic' | 'check' | 'relief' | 'regulator';
    state?: 'NO' | 'NC' | 'NONE';
    color?: string;
    rotation?: number;
    label?: string;
    sublabel?: string;
}> = ({ x, y, type = 'manual', state = 'NONE', color = 'black', rotation = 0, label, sublabel }) => {
    const size = 24;
    const isNC = state === 'NC';
    const fill = isNC ? color : 'white';

    let symbol = null;

    if (type === 'check') {
        symbol = (
            <svg width={size} height={size} viewBox="0 0 24 24" style={{ overflow: 'visible' }}>
                <path d="M 4,4 L 20,12 L 4,20 Z" fill="white" stroke={color} strokeWidth="2" />
                <line x1="20" y1="4" x2="20" y2="20" stroke={color} strokeWidth="2" />
            </svg>
        );
    } else if (type === 'relief') {
        symbol = (
            <svg width={size} height={size * 1.5} viewBox="0 0 24 36" style={{ overflow: 'visible' }}>
                <path d="M 4,24 L 20,24 L 12,12 Z" fill="white" stroke={color} strokeWidth="2" />
                <line x1="12" y1="12" x2="12" y2="0" stroke={color} strokeWidth="2" />
                <path d="M 12,12 L 8,8 L 16,4 L 8,0" fill="none" stroke={color} strokeWidth="1.5" />
                <line x1="6" y1="0" x2="18" y2="0" stroke={color} strokeWidth="2" />
                <polygon points="10,0 14,0 12,-4" fill={color} />
            </svg>
        );
    } else if (type === 'regulator') {
        symbol = (
            <svg width={size * 1.5} height={size * 1.5} viewBox="0 0 36 36" style={{ overflow: 'visible' }}>
                {/* Bowtie */}
                <path d="M 4,24 L 12,18 L 4,12 Z M 20,24 L 12,18 L 20,12 Z" fill={fill} stroke={color} strokeWidth="2" />
                {/* Dome */}
                <path d="M 6,12 Q 12,-4 18,12 Z" fill="white" stroke={color} strokeWidth="2" />
                <line x1="12" y1="12" x2="12" y2="4" stroke={color} strokeWidth="2" />
                {/* Dashed line */}
                <line x1="12" y1="4" x2="30" y2="4" stroke={color} strokeWidth="1.5" strokeDasharray="2,2" />
                <line x1="30" y1="4" x2="30" y2="18" stroke={color} strokeWidth="1.5" strokeDasharray="2,2" />
            </svg>
        );
    } else {
        // Standard bowtie base
        symbol = (
            <svg width={size * 1.5} height={size} viewBox="0 0 36 24" style={{ overflow: 'visible' }}>
                <path d="M 4,20 L 18,12 L 4,4 Z M 32,20 L 18,12 L 32,4 Z" fill={fill} stroke={color} strokeWidth="2" />
                
                {type === 'solenoid' && (
                    <g>
                        <rect x="12" y="-12" width="12" height="16" fill="white" stroke={color} strokeWidth="1.5" />
                        <text x="18" y="-1" fontSize="10" textAnchor="middle" fill={color} fontFamily="Arial" fontWeight="bold">S</text>
                        <line x1="18" y1="4" x2="18" y2="12" stroke={color} strokeWidth="1.5" />
                    </g>
                )}
                {type === 'pneumatic' && (
                    <g>
                        <path d="M 8,-4 L 28,-4 L 28,4 L 8,4 Z" fill="white" stroke={color} strokeWidth="1.5" />
                        <path d="M 8,-4 Q 18,-16 28,-4 Z" fill="white" stroke={color} strokeWidth="1.5" />
                        <line x1="18" y1="4" x2="18" y2="12" stroke={color} strokeWidth="1.5" />
                    </g>
                )}
                {type === 'manual' && (
                    <g>
                        <line x1="10" y1="-4" x2="26" y2="-4" stroke={color} strokeWidth="2" />
                        <line x1="18" y1="-4" x2="18" y2="12" stroke={color} strokeWidth="2" />
                    </g>
                )}
            </svg>
        );
    }

    return (
        <div style={{
            position: 'absolute',
            left: x, top: y,
            transform: `translate(-50%, -50%)`,
            zIndex: 3,
            display: 'flex',
            flexDirection: 'column',
            alignItems: 'center',
        }}>
            <div style={{ transform: `rotate(${rotation}deg)` }}>
                {symbol}
            </div>
            {label && <div style={{ fontSize: 10, fontWeight: 'bold', color: 'black', marginTop: 2, whiteSpace: 'nowrap', backgroundColor: 'rgba(255,255,255,0.8)', padding: '0 2px' }}>{label}</div>}
            {sublabel && <div style={{ fontSize: 9, color: 'red', whiteSpace: 'nowrap', backgroundColor: 'rgba(255,255,255,0.8)', padding: '0 2px' }}>{sublabel}</div>}
        </div>
    );
};

// --- Other Components ---
export const Filter: React.FC<{ x: number; y: number; label: string; sublabel?: string; rotation?: number }> = ({ x, y, label, sublabel, rotation = 0 }) => (
    <div style={{
        position: 'absolute', left: x, top: y,
        transform: `translate(-50%, -50%)`,
        display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center',
        zIndex: 3,
    }}>
        <div style={{ transform: `rotate(${rotation}deg)` }}>
            <svg width="30" height="30" viewBox="0 0 30 30" style={{ overflow: 'visible' }}>
                <polygon points="15,0 30,15 15,30 0,15" fill="white" stroke="black" strokeWidth="2" />
                <line x1="0" y1="15" x2="30" y2="15" stroke="black" strokeWidth="1.5" strokeDasharray="3,3" />
            </svg>
        </div>
        <div style={{ position: 'absolute', top: -15, fontSize: 10, fontWeight: 'bold', color: 'black', whiteSpace: 'nowrap', backgroundColor: 'rgba(255,255,255,0.8)' }}>{label}</div>
        {sublabel && <div style={{ position: 'absolute', bottom: -15, fontSize: 9, color: 'black', whiteSpace: 'nowrap', backgroundColor: 'rgba(255,255,255,0.8)' }}>{sublabel}</div>}
    </div>
);

export const Pump: React.FC<{ x: number; y: number; label: string }> = ({ x, y, label }) => (
    <div style={{
        position: 'absolute', left: x, top: y,
        transform: `translate(-50%, -50%)`,
        display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center',
        zIndex: 3,
    }}>
        <svg width="30" height="30" viewBox="0 0 30 30" style={{ overflow: 'visible' }}>
            <circle cx="15" cy="15" r="15" fill="white" stroke="black" strokeWidth="2" />
            <polygon points="10,8 22,15 10,22" fill="none" stroke="black" strokeWidth="2" />
        </svg>
        <div style={{ position: 'absolute', top: -15, fontSize: 10, fontWeight: 'bold', color: 'black', whiteSpace: 'nowrap', backgroundColor: 'rgba(255,255,255,0.8)' }}>{label}</div>
    </div>
);

export const Coil: React.FC<{ x: number; y: number }> = ({ x, y }) => (
    <div style={{
        position: 'absolute', left: x, top: y,
        transform: `translate(-50%, -50%)`,
        zIndex: 3,
    }}>
        <svg width="40" height="20" viewBox="0 0 40 20" style={{ overflow: 'visible' }}>
            <path d="M 0,10 Q 5,0 10,10 T 20,10 T 30,10 T 40,10" fill="none" stroke="green" strokeWidth="3" />
        </svg>
    </div>
);

export const Funnel: React.FC<{ x: number; y: number }> = ({ x, y }) => (
    <div style={{
        position: 'absolute', left: x, top: y,
        transform: `translate(-50%, -50%)`,
        zIndex: 3,
    }}>
        <svg width="30" height="40" viewBox="0 0 30 40" style={{ overflow: 'visible' }}>
            <polygon points="0,0 30,0 20,20 10,20" fill="white" stroke="black" strokeWidth="2" />
            <line x1="15" y1="20" x2="15" y2="40" stroke="black" strokeWidth="2" />
        </svg>
    </div>
);

export const Engine: React.FC<{ x: number; y: number }> = ({ x, y }) => (
    <div style={{ position: 'absolute', left: x, top: y, zIndex: 2 }}>
        {/* Injector */}
        <div style={{
            position: 'absolute', left: 0, top: 0,
            width: 40, height: 120,
            backgroundColor: 'white',
            border: '2px solid black',
            display: 'flex', alignItems: 'center', justifyContent: 'center',
            writingMode: 'vertical-rl',
            textOrientation: 'mixed',
            fontSize: 12, fontWeight: 'bold', color: 'black'
        }}>
            INJECTOR
        </div>
        {/* Chamber */}
        <div style={{
            position: 'absolute', left: 40, top: 10,
            width: 100, height: 100,
            backgroundColor: 'white',
            border: '2px solid black',
            borderLeft: 'none',
            display: 'flex', alignItems: 'center', justifyContent: 'center',
            fontSize: 14, fontWeight: 'bold', color: 'black'
        }}>
            CHAMBER
        </div>
        {/* Nozzle */}
        <svg width="80" height="120" style={{ position: 'absolute', left: 140, top: 0, overflow: 'visible' }}>
            <path d="M 0,10 L 80,-20 L 80,140 L 0,110 Z" fill="white" stroke="black" strokeWidth="2" />
        </svg>
    </div>
);
