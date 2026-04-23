import React, { useRef, useState, useEffect, useCallback } from 'react';
import { useBackend } from '../context/BackendContext';
import { ValveToggle } from './ValveToggle';
import { Valve } from './Valve';
import { ValveState } from './ValveState';

/**
 * MissionControlRanger
 * ------------------------------------------------------------
 * Ranger mission-control dashboard styled after the Atlas UI.
 *
 * Layout (base 2560 x 1440):
 *   - Title bar                         0-120
 *   - VEHICLE schematic (top)          120-1020
 *   - GSE schematic (bottom-left)     1040-1400  x 0-1100
 *   - PAD (LC cells)                  1040-1120  x 1110-2040
 *   - LOAD CELL / COUNTDOWN (bottom)  1125-1375  x 1120-2040
 *   - Right control sidebar              0-1440  x 2060-2560
 */

const BASE_W = 2560;
const BASE_H = 1440;

// --- Color palette (matches the Ranger P&ID + Atlas dark aesthetic) ---
const C = {
    bg: '#000000',
    border: '#ffffff',
    gn2: '#1bb84a',          // green - nitrogen / pressurant
    lox: '#2a6ef5',          // blue  - oxidizer
    fuel: '#d82020',         // red   - kerosene / fuel
    panel: '#0a0a0a',
    panelBorder: '#c9c9c9',
    text: '#ffffff',
    readout: '#2ad12f',
    unit: '#69eef0',
    dim: '#7f7f7f',
    lineGrey: '#d0d0d0',
};

const FALLBACK_SEQUENCES = [
    { id: 'ignition', name: 'ignition', startTime: 6, tMin: -6, tMax: 9 },
];

/* ============================================================ */
/*  Reusable schematic primitives (SVG)                          */
/* ============================================================ */

const Wire: React.FC<{
    points: [number, number][];
    color?: string;
    width?: number;
    dashed?: boolean;
}> = ({ points, color = C.gn2, width = 3, dashed = false }) => {
    const d = points
        .map((p, i) => (i === 0 ? `M ${p[0]} ${p[1]}` : `L ${p[0]} ${p[1]}`))
        .join(' ');
    return (
        <path
            d={d}
            stroke={color}
            strokeWidth={width}
            fill="none"
            strokeDasharray={dashed ? '6 4' : undefined}
            strokeLinecap="square"
            strokeLinejoin="miter"
        />
    );
};

/** Bowtie-shaped valve symbol (manual/solenoid/pneumatic/relief/check/reg). */
const SvgValve: React.FC<{
    x: number; y: number;
    type?: 'manual' | 'solenoid' | 'pneumatic' | 'check' | 'relief' | 'regulator' | 'quickDisconnect';
    state?: 'NO' | 'NC' | '';
    color?: string;
    rotation?: number;
    label?: string;
    sublabel?: string;
    sublabelColor?: string;
    strike?: boolean; // red "X" overlay, used for SV-003 / SV-004 per reference
    w?: number;
}> = ({
    x, y,
    type = 'manual',
    state = '',
    color = C.text,
    rotation = 0,
    label,
    sublabel,
    sublabelColor,
    strike = false,
    w = 36,
}) => {
        const fill = '#000000';
        const stroke = color;
        const bowW = w;
        const bowH = w * 0.6;

        const labelOffsetY = bowH / 2 + 8;

        return (
            <g transform={`translate(${x} ${y})`}>
                <g transform={`rotate(${rotation})`}>
                    {/* Bowtie */}
                    <path
                        d={`M ${-bowW / 2} ${-bowH / 2} L 0 0 L ${-bowW / 2} ${bowH / 2} Z
                M ${bowW / 2} ${-bowH / 2} L 0 0 L ${bowW / 2} ${bowH / 2} Z`}
                        fill={fill}
                        stroke={stroke}
                        strokeWidth={2}
                    />

                    {/* Actuator hats */}
                    {type === 'solenoid' && (
                        <g>
                            <rect x={-9} y={-bowH / 2 - 18} width={18} height={16}
                                fill={fill} stroke={stroke} strokeWidth={1.8} />
                            <text x={0} y={-bowH / 2 - 6} fontSize={11}
                                fill={stroke} textAnchor="middle" fontFamily="Arial" fontWeight="bold">S</text>
                            <line x1={0} y1={-bowH / 2} x2={0} y2={-bowH / 2 - 2}
                                stroke={stroke} strokeWidth={1.5} />
                        </g>
                    )}
                    {type === 'pneumatic' && (
                        <g>
                            <path d={`M -14 -${bowH / 2 + 18} L 14 -${bowH / 2 + 18}
                    L 14 -${bowH / 2} L -14 -${bowH / 2} Z`}
                                fill={fill} stroke={stroke} strokeWidth={1.8} />
                            <line x1={-14} y1={-bowH / 2 - 18} x2={14} y2={-bowH / 2}
                                stroke={stroke} strokeWidth={1.5} />
                            <line x1={14} y1={-bowH / 2 - 18} x2={-14} y2={-bowH / 2}
                                stroke={stroke} strokeWidth={1.5} />
                        </g>
                    )}
                    {type === 'manual' && (
                        <g>
                            <line x1={-12} y1={-bowH / 2 - 14} x2={12} y2={-bowH / 2 - 14}
                                stroke={stroke} strokeWidth={2.5} />
                            <line x1={0} y1={-bowH / 2 - 14} x2={0} y2={-bowH / 2}
                                stroke={stroke} strokeWidth={2} />
                        </g>
                    )}
                    {type === 'check' && (
                        <g>
                            <line x1={bowW / 2} y1={-bowH / 2 - 2} x2={bowW / 2} y2={bowH / 2 + 2}
                                stroke={stroke} strokeWidth={2.5} />
                        </g>
                    )}
                    {type === 'relief' && (
                        <g>
                            <line x1={0} y1={-bowH / 2} x2={0} y2={-bowH / 2 - 18}
                                stroke={stroke} strokeWidth={2} />
                            <path d={`M -10 -${bowH / 2 + 18} L 10 -${bowH / 2 + 18}
                    L 0 -${bowH / 2 + 32} Z`} fill={fill} stroke={stroke} strokeWidth={2} />
                            <polygon points={`-3,-${bowH / 2 + 32} 3,-${bowH / 2 + 32} 0,-${bowH / 2 + 40}`}
                                fill={stroke} />
                        </g>
                    )}
                    {type === 'regulator' && (
                        <g>
                            <line x1={0} y1={-bowH / 2} x2={0} y2={-bowH / 2 - 12}
                                stroke={stroke} strokeWidth={2} />
                            <path d={`M -14 -${bowH / 2 + 12 - 0} Q 0 -${bowH / 2 + 12 - 28} 14 -${bowH / 2 + 12}`}
                                fill="none" stroke={stroke} strokeWidth={2} />
                            <rect x={-16} y={-bowH / 2 - 16} width={32} height={4}
                                fill={fill} stroke={stroke} strokeWidth={1.5} />
                        </g>
                    )}
                    {type === 'quickDisconnect' && (
                        <g>
                            <rect x={-bowW / 2 - 4} y={-bowH / 2} width={bowW + 8} height={bowH}
                                fill={fill} stroke={stroke} strokeWidth={2} />
                            <line x1={0} y1={-bowH / 2} x2={0} y2={bowH / 2}
                                stroke={stroke} strokeWidth={2} strokeDasharray="3 2" />
                        </g>
                    )}

                    {strike && (
                        <g stroke="#ff2a2a" strokeWidth={2.6} strokeLinecap="round">
                            <line x1={-bowW / 2 - 2} y1={-bowH / 2 - 2}
                                x2={bowW / 2 + 2} y2={bowH / 2 + 2} />
                            <line x1={bowW / 2 + 2} y1={-bowH / 2 - 2}
                                x2={-bowW / 2 - 2} y2={bowH / 2 + 2} />
                        </g>
                    )}
                </g>

                {label && (
                    <text x={0} y={labelOffsetY + 10} fill={color}
                        fontSize={11} textAnchor="middle" fontFamily="Arial" fontWeight="bold">
                        {label}
                    </text>
                )}
                {state && (
                    <text x={0} y={labelOffsetY + 22} fill={color}
                        fontSize={10} textAnchor="middle" fontFamily="Arial">
                        {state}
                    </text>
                )}
                {sublabel && (
                    <text x={0} y={labelOffsetY + (state ? 34 : 22)}
                        fill={sublabelColor || color}
                        fontSize={10} textAnchor="middle" fontFamily="Arial">
                        {sublabel}
                    </text>
                )}
            </g>
        );
    };

/** Small circular gauge/sensor bubble (PT-xxx, TC-xxx, FT-xxx). */
const GaugeBubble: React.FC<{
    x: number; y: number;
    label: string;
    sublabel?: string;
    r?: number;
    color?: string;
}> = ({ x, y, label, sublabel, r = 14, color = C.text }) => (
    <g transform={`translate(${x} ${y})`}>
        <circle r={r} fill="#000000" stroke={color} strokeWidth={1.6} />
        <text y={sublabel ? -2 : 3} fontSize={10} fill={color}
            textAnchor="middle" fontFamily="Arial" fontWeight="bold">{label}</text>
        {sublabel && (
            <text y={9} fontSize={9} fill={color}
                textAnchor="middle" fontFamily="Arial">{sublabel}</text>
        )}
    </g>
);

/** Filter diamond with inner dashed divider. */
const SvgFilter: React.FC<{
    x: number; y: number; label: string; sublabel?: string; w?: number;
}> = ({ x, y, label, sublabel, w = 34 }) => (
    <g transform={`translate(${x} ${y})`}>
        <polygon points={`${-w / 2},0 0,${-w / 2} ${w / 2},0 0,${w / 2}`}
            fill="#000000" stroke={C.text} strokeWidth={2} />
        <line x1={-w / 2} y1={0} x2={w / 2} y2={0}
            stroke={C.text} strokeWidth={1.2} strokeDasharray="3 2" />
        <text y={-w / 2 - 8} fontSize={11} fill={C.text}
            textAnchor="middle" fontFamily="Arial" fontWeight="bold">{label}</text>
        {sublabel && (
            <text y={w / 2 + 14} fontSize={10} fill={C.text}
                textAnchor="middle" fontFamily="Arial">{sublabel}</text>
        )}
    </g>
);

/** Pump symbol. */
const SvgPump: React.FC<{ x: number; y: number; label: string }> = ({ x, y, label }) => (
    <g transform={`translate(${x} ${y})`}>
        <circle r={18} fill="#000000" stroke={C.text} strokeWidth={2} />
        <polygon points="-8,-10 12,0 -8,10" fill="none" stroke={C.text} strokeWidth={2} />
        <text y={-26} fontSize={11} fill={C.text} textAnchor="middle"
            fontFamily="Arial" fontWeight="bold">{label}</text>
    </g>
);

/** Coil for heat exchanger / filter element. */
const SvgCoil: React.FC<{ x: number; y: number; color?: string; w?: number }> = ({ x, y, color = C.gn2, w = 60 }) => (
    <g transform={`translate(${x} ${y})`}>
        <path d={`M ${-w / 2} 0
            q 5 -12 10 0 t 10 0 t 10 0 t 10 0 t 10 0 t 10 0`}
            fill="none" stroke={color} strokeWidth={3} />
    </g>
);

/** Vertical tank with colored body + name stack. */
const SvgTank: React.FC<{
    x: number; y: number;
    w: number; h: number;
    color: string;
    lines: string[];
    horizontal?: boolean;
}> = ({ x, y, w, h, color, lines, horizontal = false }) => {
    const rx = horizontal ? h / 2 : w / 2;
    const ry = horizontal ? h / 2 : w / 2;
    return (
        <g transform={`translate(${x} ${y})`}>
            <rect x={-w / 2} y={-h / 2} width={w} height={h}
                rx={rx} ry={ry} fill={color} stroke={C.border} strokeWidth={1.5} />
            <g fontFamily="Arial" textAnchor="middle" fill={C.text} fontWeight="bold">
                {lines.map((l, i) => (
                    <text key={i} y={(i - (lines.length - 1) / 2) * 14} fontSize={12}>{l}</text>
                ))}
            </g>
        </g>
    );
};

/** Load-cell style pad circle (used for LC/1..LC/4 in PAD section). */
const SvgLoadCellIcon: React.FC<{ x: number; y: number; label: string }> = ({ x, y, label }) => (
    <g transform={`translate(${x} ${y})`}>
        <circle r={22} fill="#000000" stroke={C.text} strokeWidth={2} />
        <text y={-2} fontSize={12} fill={C.text} textAnchor="middle"
            fontFamily="Arial" fontWeight="bold">LC</text>
        <text y={12} fontSize={11} fill={C.text} textAnchor="middle"
            fontFamily="Arial">{label}</text>
    </g>
);

/* ============================================================ */
/*  Live gauge readout (HTML, uses BackendContext)              */
/* ============================================================ */

const LiveReadout: React.FC<{
    x: number; y: number; name: string; unit?: string; w?: number;
}> = ({ x, y, name, unit = 'PSI', w = 160 }) => {
    const { gauges } = useBackend();
    const v = gauges[name];
    const value = (v !== undefined && v !== null) ? v.toFixed(1) : 'N/A';
    return (
        <div style={{
            position: 'absolute', left: x, top: y, width: w, height: 22,
            border: `1px solid ${C.panelBorder}`, background: '#000',
            display: 'flex', alignItems: 'center', padding: '0 6px',
            fontFamily: 'Arial', fontSize: 13,
            boxSizing: 'border-box',
        }}>
            <span style={{ color: C.text }}>{name}:</span>
            <span style={{
                color: C.readout, marginLeft: 'auto', marginRight: 4,
                fontFamily: 'Consolas, monospace'
            }}>{value}</span>
            <span style={{ color: C.unit }}>{unit}</span>
        </div>
    );
};

/* ============================================================ */
/*  MAIN COMPONENT                                                */
/* ============================================================ */

const Dashboard: React.FC = () => {
    const { gauges, timer, serverStatus, abort, actuate, startCountdown, stopSequence, sequences, refreshSequences } = useBackend();
    const wrapperRef = useRef<HTMLDivElement>(null);
    const [scale, setScale] = useState(1);
    const [selectedSequence, setSelectedSequence] = useState(FALLBACK_SEQUENCES[0].id);
    const availableSequences = sequences.length > 0 ? sequences : FALLBACK_SEQUENCES;

    const handleResize = useCallback(() => {
        if (!wrapperRef.current) return;
        const { clientWidth: w, clientHeight: h } = wrapperRef.current;
        setScale(Math.min(w / BASE_W, h / BASE_H));
    }, []);

    useEffect(() => {
        handleResize();
        window.addEventListener('resize', handleResize);
        return () => window.removeEventListener('resize', handleResize);
    }, [handleResize]);

    useEffect(() => {
        if (!availableSequences.some(s => s.id === selectedSequence)) {
            setSelectedSequence(availableSequences[0]?.id ?? '');
        }
    }, [availableSequences, selectedSequence]);

    const lc = (name: string) => gauges[name]?.toFixed(1) ?? 'N/A';

    return (
        <div
            ref={wrapperRef}
            style={{
                width: '100vw',
                height: '100vh',
                backgroundColor: C.bg,
                overflow: 'hidden',
                position: 'relative',
            }}
        >
            <div
                style={{
                    width: BASE_W,
                    height: BASE_H,
                    transformOrigin: 'top left',
                    transform: `scale(${scale})`,
                    position: 'absolute',
                    top: 0, left: 0,
                    fontFamily: 'Arial, sans-serif',
                    color: C.text,
                    background: C.bg,
                }}
            >
                {/* ================ TITLE BAR ================ */}
                {/* Full-width thin red stripe at the very top, matching reference */}
                <div style={{
                    position: 'absolute', top: 0, left: 0, width: BASE_W, height: 5,
                    background: '#b91818',
                }} />
                <img
                    src="/assets/logo%20white.png"
                    alt="LPL"
                    style={{
                        position: 'absolute', left: 40, top: 22,
                        width: 100, height: 74, objectFit: 'contain',
                    }}
                    onError={(e) => { (e.currentTarget as HTMLImageElement).style.display = 'none'; }}
                />
                <div style={{
                    position: 'absolute', left: 160, top: 32,
                    fontSize: 54, letterSpacing: 1, fontWeight: 500,
                    lineHeight: '60px',
                }}>
                    MISSION CONTROL - RANGER
                </div>

                {/* ================ SCHEMATIC SVG ================ */}
                <svg
                    width={2090}
                    height={1020}
                    viewBox="0 0 2090 1020"
                    style={{ position: 'absolute', left: 0, top: 120 }}
                >
                    {/* Section frames */}
                    <rect x={10} y={10} width={2070} height={800}
                        fill="none" stroke={C.panelBorder} strokeWidth={1.2} />
                    <rect x={10} y={830} width={1300} height={180}
                        fill="none" stroke={C.panelBorder} strokeWidth={1.2} />
                    <rect x={1320} y={830} width={760} height={180}
                        fill="none" stroke={C.panelBorder} strokeWidth={1.2} />

                    {/* Section title tags */}
                    <g>
                        <rect x={18} y={14} width={110} height={26} fill="#000"
                            stroke={C.panelBorder} />
                        <text x={73} y={33} fill={C.text} fontSize={17}
                            textAnchor="middle" fontFamily="Arial" fontWeight="bold">VEHICLE</text>
                    </g>
                    <g>
                        <rect x={18} y={834} width={70} height={24} fill="#000"
                            stroke={C.panelBorder} />
                        <text x={53} y={852} fill={C.text} fontSize={16}
                            textAnchor="middle" fontFamily="Arial" fontWeight="bold">GSE</text>
                    </g>
                    <g>
                        <rect x={1328} y={834} width={70} height={24} fill="#000"
                            stroke={C.panelBorder} />
                        <text x={1363} y={852} fill={C.text} fontSize={16}
                            textAnchor="middle" fontFamily="Arial" fontWeight="bold">PAD</text>
                    </g>

                    {/* ================================================ */}
                    {/*               VEHICLE SCHEMATIC                    */}
                    {/* ================================================ */}

                    {/* ---- GN2 pressurant bottle (top-left) ---- */}
                    <SvgTank x={120} y={360} w={130} h={190} color={C.gn2}
                        lines={['GN2', '6X 6.8L', '4300 MEOP']} />

                    {/* Main pressurant supply callout (left of GN2) */}
                    <text x={40} y={250} fill={C.text} fontSize={13}
                        fontFamily="Arial" fontWeight="bold">MAIN</text>
                    <text x={40} y={266} fill={C.text} fontSize={13}
                        fontFamily="Arial" fontWeight="bold">PRESSURANT</text>
                    <text x={40} y={282} fill={C.text} fontSize={13}
                        fontFamily="Arial" fontWeight="bold">SUPPLY</text>

                    {/* Vertical main N2 trunk from GN2 up to VEHICLE top-stream */}
                    <Wire points={[[185, 265], [185, 200], [260, 200], [260, 400]]} color={C.gn2} />
                    {/* GN2 -> SV-002/PRV-001 branch */}
                    <Wire points={[[185, 265], [185, 330]]} color={C.gn2} />
                    {/* QD-1 & BV-001 down from GN2 */}
                    <Wire points={[[120, 455], [120, 600]]} color={C.gn2} />
                    <Wire points={[[185, 455], [185, 470], [185, 500]]} color={C.gn2} />

                    {/* BV-001 (manual, below GN2) */}
                    <SvgValve x={185} y={480} type="manual" color={C.gn2}
                        rotation={90} label="BV-001" />
                    {/* QD-1 (manual, bottom-left) */}
                    <SvgValve x={120} y={600} type="quickDisconnect" color={C.gn2}
                        rotation={90} label="QD-1" sublabel="(manual)" />

                    {/* SV-002 NO (upper left branch) */}
                    <SvgValve x={185} y={332} type="solenoid" color={C.gn2}
                        rotation={0} label="SV-002" sublabel="NO" />
                    {/* PRV-001 (upper relief) */}
                    <SvgValve x={260} y={260} type="relief" color={C.gn2}
                        label="PRV-001" />
                    <Wire points={[[260, 228], [260, 190]]} color={C.gn2} />

                    {/* FLT-001 filter */}
                    <SvgFilter x={320} y={400} label="FLT-001" sublabel="60 micron" />
                    <Wire points={[[260, 400], [337, 400]]} color={C.gn2} />
                    <Wire points={[[303, 400], [370, 400]]} color={C.gn2} />

                    {/* SV-001 NC (after filter) */}
                    <SvgValve x={410} y={400} type="solenoid" color={C.gn2}
                        label="SV-001" sublabel="NC" />

                    {/* Manifold split: REG-001 (top) / REG-003 (middle) / REG-002 (bottom) */}
                    <Wire points={[[432, 400], [520, 400]]} color={C.gn2} />
                    <Wire points={[[520, 400], [520, 230]]} color={C.gn2} />
                    <Wire points={[[520, 400], [520, 580]]} color={C.gn2} />

                    {/* REG-001 (top path, feeds LOX ullage) */}
                    <SvgValve x={540} y={230} type="regulator" color={C.gn2}
                        rotation={0} label="REG-001" />
                    <Wire points={[[520, 230], [640, 230]]} color={C.gn2} />
                    {/* REG-003 (middle, cross-flow manifold) */}
                    <SvgValve x={540} y={400} type="regulator" color={C.gn2}
                        label="REG-003" sublabel="NC" />
                    <Wire points={[[562, 400], [1100, 400]]} color={C.gn2} />
                    {/* REG-002 (bottom path, feeds JET-A ullage) */}
                    <SvgValve x={540} y={580} type="regulator" color={C.gn2}
                        label="REG-002" />
                    <Wire points={[[520, 580], [640, 580]]} color={C.gn2} />

                    {/* SV-003 (NC Temp) on top line (strike-through per ref) */}
                    <SvgValve x={680} y={230} type="solenoid" color={C.gn2}
                        label="SV-003" sublabel="NC TEMP" sublabelColor="#ff6a6a" strike />
                    {/* CV-001 check valve after SV-003 */}
                    <SvgValve x={780} y={230} type="check" color={C.gn2}
                        label="CV-001" />
                    <Wire points={[[703, 230], [758, 230]]} color={C.gn2} />
                    <Wire points={[[803, 230], [870, 230]]} color={C.gn2} />

                    {/* BV-002 NO (before LOx tank) */}
                    <SvgValve x={900} y={160} type="manual" color={C.gn2}
                        rotation={90} label="BV-002" sublabel="NO" />
                    <Wire points={[[870, 230], [870, 160], [877, 160]]} color={C.gn2} />
                    <Wire points={[[921, 160], [990, 160]]} color={C.gn2} />
                    {/* PRV-002 */}
                    <SvgValve x={840} y={80} type="relief" color={C.gn2}
                        label="PRV-002" />
                    <Wire points={[[840, 48], [840, 30]]} color={C.gn2} />
                    <Wire points={[[870, 160], [870, 115], [840, 115], [840, 112]]} color={C.gn2} />

                    {/* LOx tank */}
                    <SvgTank x={1020} y={210} w={180} h={130} color={C.lox}
                        lines={['LOx', '12 GAL', 'MEOP: 800 PSIG']} horizontal />

                    {/* LOx ullage top outlet -> GV-101 manual (to Dewar) */}
                    <Wire points={[[1050, 146], [1050, 80], [1140, 80]]} color={C.lox} width={3} />
                    <SvgValve x={1170} y={80} type="manual" color={C.lox}
                        rotation={90} label="GV-101" sublabel="Manual" />
                    <Wire points={[[1198, 80], [1260, 80]]} color={C.lox} />
                    <text x={1260} y={60} fill={C.text} fontSize={11}
                        fontFamily="Arial">To Dewar</text>

                    {/* LOx bottom outlet -> PBV-101 -> TV-101 -> injector */}
                    <Wire points={[[1110, 275], [1230, 275]]} color={C.lox} />
                    <SvgValve x={1255} y={275} type="pneumatic" color={C.lox}
                        label="PBV-101" sublabel="NC" />
                    <Wire points={[[1281, 275], [1350, 275]]} color={C.lox} />
                    <SvgValve x={1380} y={275} type="manual" color={C.lox}
                        label="TV-101" />
                    <Wire points={[[1405, 275], [1640, 275]]} color={C.lox} />

                    {/* CV-004 on LOx injector line */}
                    <SvgValve x={1660} y={275} type="check" color={C.lox}
                        label="CV-004" />
                    <Wire points={[[1685, 275], [1820, 275]]} color={C.lox} />
                    <Wire points={[[1820, 275], [1820, 370]]} color={C.lox} />

                    {/* GN2 purge cross into LOx/JET-A injector lines */}
                    <Wire points={[[1100, 400], [1100, 275], [1230, 275]]} color={C.gn2} dashed />
                    <Wire points={[[1100, 400], [1100, 580], [1230, 580]]} color={C.gn2} dashed />

                    {/* Flow arrows on main injector runs (thin triangles) */}
                    <polygon points="1440,273 1440,277 1444,275"
                        fill={C.lox} stroke={C.lox} strokeWidth={1.5} />
                    <polygon points="1440,633 1440,637 1444,635"
                        fill={C.fuel} stroke={C.fuel} strokeWidth={1.5} />
                    <polygon points="1700,273 1700,277 1704,275"
                        fill={C.lox} stroke={C.lox} strokeWidth={1.5} />
                    <polygon points="1700,633 1700,637 1704,635"
                        fill={C.fuel} stroke={C.fuel} strokeWidth={1.5} />

                    {/* OX PURGE / FUEL PURGE callouts near the N2 cross-tap */}
                    <text x={1110} y={310} fill={C.gn2} fontSize={10}
                        fontFamily="Arial" fontWeight="bold">OX PURGE</text>
                    <text x={1110} y={560} fill={C.gn2} fontSize={10}
                        fontFamily="Arial" fontWeight="bold">FUEL PURGE</text>

                    {/* --- JET-A side (bottom half) --- */}
                    <SvgValve x={680} y={580} type="solenoid" color={C.gn2}
                        label="SV-004" sublabel="NC TEMP" sublabelColor="#ff6a6a" strike />
                    <SvgValve x={780} y={580} type="check" color={C.gn2}
                        label="CV-002" />
                    <Wire points={[[703, 580], [758, 580]]} color={C.gn2} />
                    <Wire points={[[803, 580], [870, 580], [870, 680]]} color={C.gn2} />

                    {/* SV-005 NO (bottom left relief branch into tank) */}
                    <SvgValve x={780} y={700} type="solenoid" color={C.gn2}
                        label="SV-005" sublabel="NO" />
                    <Wire points={[[758, 700], [680, 700]]} color={C.gn2} />
                    <Wire points={[[870, 700], [870, 740]]} color={C.gn2} />
                    {/* PRV-003 relief */}
                    <SvgValve x={680} y={760} type="relief" color={C.gn2} rotation={180}
                        label="PRV-003" />

                    {/* JET-A tank */}
                    <SvgTank x={1020} y={700} w={180} h={130} color={C.fuel}
                        lines={['JET-A', '12 GAL', 'MEOP: 900 PSIG']} horizontal />

                    {/* BV-201 manual (to Kero Funnel) */}
                    <Wire points={[[1110, 765], [1110, 830]]} color={C.fuel} />
                    <SvgValve x={1110} y={855} type="manual" color={C.fuel} rotation={90}
                        label="BV-201" sublabel="Manual" />
                    <text x={1110} y={935} fill={C.text} fontSize={11}
                        fontFamily="Arial" textAnchor="middle">To Kero</text>
                    <text x={1110} y={948} fill={C.text} fontSize={11}
                        fontFamily="Arial" textAnchor="middle">Funnel</text>

                    {/* JET-A outlet -> PBV-201 -> TV-201 -> chamber */}
                    <Wire points={[[1110, 635], [1230, 635]]} color={C.fuel} />
                    <SvgValve x={1255} y={635} type="pneumatic" color={C.fuel}
                        label="PBV-201" sublabel="NC" />
                    <Wire points={[[1281, 635], [1350, 635]]} color={C.fuel} />
                    <SvgValve x={1380} y={635} type="manual" color={C.fuel}
                        label="TV-201" />
                    <Wire points={[[1405, 635], [1640, 635]]} color={C.fuel} />

                    {/* SV-006 NO (middle cross-flow manifold return) */}
                    <SvgValve x={1020} y={440} type="solenoid" color={C.gn2}
                        label="SV-006" sublabel="NO" />
                    <Wire points={[[1100, 400], [1100, 440], [1043, 440]]} color={C.gn2} />
                    <Wire points={[[997, 440], [900, 440]]} color={C.gn2} />

                    {/* CV-005 on JET-A injector line */}
                    <SvgValve x={1660} y={635} type="check" color={C.fuel}
                        label="CV-005" />
                    <Wire points={[[1685, 635], [1820, 635]]} color={C.fuel} />
                    <Wire points={[[1820, 635], [1820, 540]]} color={C.fuel} />

                    {/* ---- INJECTOR / CHAMBER ---- */}
                    <g>
                        {/* Injector manifold stack (blue + red + orange) */}
                        <rect x={1820} y={370} width={30} height={60} fill={C.lox} stroke={C.border} />
                        <rect x={1820} y={430} width={30} height={30} fill="#e07a0a" stroke={C.border} />
                        <rect x={1820} y={460} width={30} height={80} fill={C.fuel} stroke={C.border} />
                        {/* Injector body */}
                        <rect x={1850} y={340} width={50} height={240} fill="#0d0d0d"
                            stroke={C.border} strokeWidth={2} />
                        <text x={1875} y={460} fill={C.text} fontSize={14}
                            fontFamily="Arial" fontWeight="bold"
                            textAnchor="middle" transform="rotate(-90 1875 460)">
                            INJECTOR
                        </text>
                        {/* Chamber */}
                        <rect x={1900} y={380} width={70} height={160} fill="#0d0d0d"
                            stroke={C.border} strokeWidth={2} />
                        <text x={1935} y={465} fill={C.text} fontSize={15}
                            fontFamily="Arial" fontWeight="bold" textAnchor="middle">
                            CHAMBER
                        </text>
                        {/* Nozzle */}
                        <path d="M 1970 380 L 2030 350 L 2030 570 L 1970 540 Z"
                            fill="#0d0d0d" stroke={C.border} strokeWidth={2} />
                    </g>

                    {/* ---- Sensor bubbles on VEHICLE diagram ---- */}
                    {/* PT/FT near GN2 pressurant */}
                    <GaugeBubble x={72} y={315} label="FT" sublabel="101" />
                    <GaugeBubble x={230} y={295} label="FT" sublabel="102" />
                    {/* PT002 near FLT/SV-001 */}
                    <GaugeBubble x={370} y={360} label="PT" sublabel="002" />
                    {/* PT003 between REG-001 and SV-003 */}
                    <GaugeBubble x={620} y={195} label="PT" sublabel="003" />
                    <GaugeBubble x={620} y={545} label="PT" sublabel="005" />
                    {/* FT for REG-003 cross */}
                    <GaugeBubble x={780} y={370} label="FT" sublabel="103" />
                    <GaugeBubble x={780} y={430} label="FT" sublabel="104" />
                    {/* LOx tank top/bottom */}
                    <GaugeBubble x={1115} y={140} label="PT" sublabel="101" />
                    <GaugeBubble x={1190} y={115} label="TC" sublabel="101" />
                    <GaugeBubble x={1330} y={250} label="TC" sublabel="102" />
                    <GaugeBubble x={1680} y={255} label="FT" sublabel="105" />
                    {/* JET-A tank sensors */}
                    <GaugeBubble x={1115} y={825} label="PT" sublabel="201" />
                    <GaugeBubble x={1330} y={615} label="TC" sublabel="201" />
                    <GaugeBubble x={1680} y={615} label="FT" sublabel="205" />
                    {/* Chamber & Injector sensors */}
                    <GaugeBubble x={1790} y={340} label="PT" sublabel="FO" />
                    <GaugeBubble x={1790} y={580} label="PT" sublabel="201" />
                    <GaugeBubble x={2000} y={330} label="PT" sublabel="301" />
                    <GaugeBubble x={2000} y={590} label="PT" sublabel="302" />

                    {/* ================================================ */}
                    {/*                GSE SCHEMATIC                       */}
                    {/* ================================================ */}

                    {/* GN2 cylinder (high pressure ground bottle) */}
                    <SvgTank x={90} y={940} w={100} h={140} color={C.gn2}
                        lines={['GN2', 'CYLINDER', 'SIZE 300: 49L', 'MEOP:', '2800 PSIG']} />

                    {/* GN2 -> FLTG-001 (150 micron) */}
                    <Wire points={[[140, 945], [210, 945]]} color={C.gn2} />
                    <SvgCoil x={230} y={945} color={C.gn2} w={40} />
                    <text x={230} y={920} fill={C.text} fontSize={11} textAnchor="middle"
                        fontFamily="Arial" fontWeight="bold">FLTG-001</text>
                    <text x={230} y={970} fill={C.text} fontSize={10} textAnchor="middle"
                        fontFamily="Arial">150 micron</text>

                    {/* FLTG-002 (15 micron) */}
                    <Wire points={[[250, 945], [300, 945]]} color={C.gn2} />
                    <SvgFilter x={320} y={945} label="FLTG-002" sublabel="15 micron" />

                    {/* SVG-001 NC */}
                    <Wire points={[[337, 945], [380, 945]]} color={C.gn2} />
                    <SvgValve x={405} y={945} type="solenoid" color={C.gn2}
                        label="SVG-001" sublabel="NC" />

                    {/* Pump PG-001 */}
                    <Wire points={[[427, 945], [470, 945]]} color={C.gn2} />
                    <SvgPump x={490} y={945} label="PG-001" />

                    {/* SVG-002 NO down branch */}
                    <Wire points={[[490, 965], [490, 1020]]} color={C.gn2} />
                    <SvgValve x={490} y={1020} type="solenoid" color={C.gn2} rotation={180}
                        label="SVG-002" sublabel="NO" />

                    {/* QD-1 manual out */}
                    <Wire points={[[508, 945], [600, 945]]} color={C.gn2} />
                    <SvgValve x={620} y={945} type="quickDisconnect" color={C.gn2}
                        label="QD-1" sublabel="(manual)" />
                    {/* SVG-003 NO below QD-1 */}
                    <Wire points={[[620, 970], [620, 1020]]} color={C.gn2} />
                    <SvgValve x={620} y={1020} type="solenoid" color={C.gn2} rotation={180}
                        label="SVG-003" sublabel="NO" />

                    {/* Pressure gauge bubbles in GSE */}
                    <GaugeBubble x={260} y={905} label="PT" sublabel="301" />
                    <GaugeBubble x={420} y={905} label="PT" sublabel="302" />

                    {/* ---- LOX DEWAR section ---- */}
                    <SvgTank x={790} y={950} w={130} h={150} color={C.lox}
                        lines={['LOx', 'DEWAR', '230L', 'MEOP:', '230 PSIG']} />
                    {/* From Dewar top -> FLTG-101 -> To LOX GV */}
                    <Wire points={[[790, 895], [790, 870], [870, 870]]} color={C.lox} />
                    <SvgCoil x={900} y={870} color={C.lox} w={40} />
                    <text x={900} y={845} fill={C.text} fontSize={11} textAnchor="middle"
                        fontFamily="Arial" fontWeight="bold">FLTG-101</text>
                    <text x={900} y={895} fill={C.text} fontSize={10} textAnchor="middle"
                        fontFamily="Arial">60 micron</text>
                    <Wire points={[[920, 870], [1000, 870]]} color={C.lox} />
                    <text x={1010} y={872} fill={C.text} fontSize={12}
                        fontFamily="Arial">To LOX GV</text>
                    {/* BVG-101 below Dewar */}
                    <Wire points={[[790, 1025], [790, 1000]]} color={C.lox} />
                    <SvgValve x={790} y={1020} type="manual" color={C.lox}
                        rotation={90} label="BVG-101" />

                    {/* JET-A SUPPLY (lifted head tank) */}
                    <SvgTank x={1100} y={955} w={110} h={150} color={C.fuel}
                        lines={['JET-A', 'SUPPLY', '(Lifted to', 'provide head', 'pressure)']} />
                    {/* To Kero F/D via funnel */}
                    <Wire points={[[1160, 1030], [1200, 1030], [1200, 1000]]} color={C.fuel} />
                    <polygon points="1190,990 1210,990 1200,1005" fill="#000" stroke={C.text} strokeWidth={1.5} />
                    <text x={1230} y={995} fill={C.text} fontSize={11}
                        fontFamily="Arial">Funnel</text>
                    <path d="M 1200 1005 q -5 10 5 14 t -5 14 t 5 14"
                        fill="none" stroke={C.fuel} strokeWidth={2.5} />
                    <text x={1140} y={995} fill={C.text} fontSize={11}
                        fontFamily="Arial" textAnchor="middle">To Kero F/D</text>

                    {/* ================================================ */}
                    {/*                PAD - LOAD CELLS                    */}
                    {/* ================================================ */}
                    <SvgLoadCellIcon x={1450} y={925} label="1" />
                    <SvgLoadCellIcon x={1560} y={925} label="2" />
                    <SvgLoadCellIcon x={1670} y={925} label="3" />
                    <SvgLoadCellIcon x={1780} y={925} label="4" />
                </svg>

                {/* Left-panel valve overlays (same pattern as frontend-react) */}
                <div style={{ position: 'absolute', left: 0, top: 0, width: BASE_W, height: BASE_H, pointerEvents: 'none' }}>
                    {/* Vehicle side */}
                    <Valve name="SV-002" x={165} y={438} width={40} height={18} />
                    <ValveState name="SV-002" x={152} y={458} />
                    <Valve name="SV-001" x={390} y={506} width={40} height={18} />
                    <ValveState name="SV-001" x={376} y={526} />
                    <Valve name="SV-003" x={660} y={336} width={40} height={18} />
                    <ValveState name="SV-003" x={646} y={356} />
                    <Valve name="SV-004" x={660} y={686} width={40} height={18} />
                    <ValveState name="SV-004" x={646} y={706} />
                    <Valve name="SV-005" x={760} y={806} width={40} height={18} />
                    <ValveState name="SV-005" x={746} y={826} />
                    <Valve name="SV-006" x={1000} y={546} width={40} height={18} />
                    <ValveState name="SV-006" x={986} y={566} />
                    <Valve name="PBV-101" x={1235} y={381} width={44} height={20} />
                    <ValveState name="PBV-101" x={1222} y={403} />
                    <Valve name="PBV-201" x={1235} y={741} width={44} height={20} />
                    <ValveState name="PBV-201" x={1222} y={763} />

                    {/* GSE side */}
                    <Valve name="SVG-001" x={385} y={1051} width={44} height={20} />
                    <ValveState name="SVG-001" x={372} y={1073} />
                    <Valve name="SVG-002" x={470} y={1126} width={44} height={20} />
                    <ValveState name="SVG-002" x={456} y={1148} />
                    <Valve name="SVG-003" x={600} y={1126} width={44} height={20} />
                    <ValveState name="SVG-003" x={586} y={1148} />
                    <Valve name="BVG-101" x={770} y={1126} width={44} height={20} />
                    <ValveState name="BVG-101" x={756} y={1148} />
                </div>

                {/* ================ LIVE READOUTS (HTML overlays) ================ */}
                {/* positions are relative to the root canvas (top-left 0,0) */}
                <div style={{ position: 'absolute', left: 0, top: 0, width: BASE_W, height: BASE_H, pointerEvents: 'none' }}>
                    <div style={{ pointerEvents: 'auto' }}>
                        {/* PT001 - main pressurant trunk */}
                        <LiveReadout x={70} y={545} name="PT001" />
                        {/* PT002 - before SV-001 */}
                        <LiveReadout x={332} y={500} name="PT002" />
                        {/* PT003 - after REG-001 (top) */}
                        <LiveReadout x={560} y={300} name="PT003" />
                        {/* PT004 - mid manifold readout */}
                        <LiveReadout x={1050} y={500} name="PT004" />
                        {/* PT005 + TC005 around SV-004 branch */}
                        <LiveReadout x={560} y={700} name="PT005" />
                        {/* PT101, TC101, FM101 - LOx injector */}
                        <LiveReadout x={1430} y={345} name="PT101" />
                        <LiveReadout x={1430} y={370} name="TC101" unit="C" />
                        <LiveReadout x={1430} y={395} name="FM101" unit="GPM" />
                        {/* PT201, TC201, FM201 - Fuel injector */}
                        <LiveReadout x={1430} y={700} name="PT201" />
                        <LiveReadout x={1430} y={725} name="TC201" unit="C" />
                        <LiveReadout x={1430} y={750} name="FM201" unit="GPM" />
                        {/* PT301, PT302 - Chamber */}
                        <LiveReadout x={1870} y={960} name="PT301" />
                        <LiveReadout x={1870} y={985} name="PT302" />
                    </div>
                </div>

                {/* ================================================ */}
                {/*            RIGHT SIDEBAR - UNIFIED PANEL           */}
                {/* ================================================ */}
                {/*
                    One single continuous vertical panel from y=0 to y=1440,
                    matching the Atlas reference (no internal box seam).
                    Layout offsets below are hand-tuned so every row sits
                    flush against the next with no airy gaps.

                    Valve cards are rendered at 208x56 (compact variant) and
                    centred with 8px gutter on both sides and 6px between the
                    two columns, giving a dense industrial look.
                */}
                {(() => {
                    const SIDEBAR_W = 470;                              // was 500
                    const GAP_X = 6;                                    // gutter between the two columns
                    const OUTER_X = 10;                                 // inset from sidebar edges
                    const CARD_W = Math.floor((SIDEBAR_W - GAP_X - OUTER_X * 2) / 2);
                    const CARD_H = 56;
                    const ROW_GAP = 2;
                    const rowY = (i: number) => 102 + i * (CARD_H + ROW_GAP);
                    const leftX = OUTER_X;
                    const rightX = OUTER_X + CARD_W + GAP_X;

                    // Layout markers (absolute y within sidebar)
                    const Y_HEADER = 0;       // VALVES
                    const Y_SUBHEAD = 40;     // OXYGEN / KEROSENE
                    const Y_ROW_PBV = rowY(0);   // PBV-101 / PBV-201        (y=102)
                    const Y_NITRO = rowY(1) + 4;                          // y=164
                    const Y_ROW_SV01 = rowY(2) - 6;                       // y=204
                    const Y_ROW_SV03 = rowY(3) - 6;                       // y=262
                    const Y_ROW_SV05 = rowY(4) - 6;                       // y=320
                    const Y_ROW_SVG01 = rowY(5) - 6;                      // y=378
                    const Y_ROW_SVG03 = rowY(6) - 6;                      // y=436
                    const Y_ROW_IGN = rowY(7) - 6;                        // y=494
                    const Y_ACTUATE = Y_ROW_IGN + CARD_H + 14;            // y=564
                    const Y_IGN_HEADER = Y_ACTUATE + 82;                  // y=646
                    const Y_UPDATE = Y_IGN_HEADER + 44;                   // y=690
                    const Y_SEL = Y_UPDATE + 70;                          // y=760
                    const Y_START = Y_SEL + 80;                           // y=840
                    const Y_STOP = Y_START + 78;                          // y=918

                    const totalH = Y_STOP + 64 + 12;                      // sidebar bottom

                    return (
                        <div style={{
                            position: 'absolute', left: BASE_W - SIDEBAR_W, top: 0,
                            width: SIDEBAR_W, height: Math.max(totalH, BASE_H),
                            background: '#000',
                            borderLeft: `1px solid ${C.border}`,
                            borderRight: `1px solid ${C.border}`,
                            borderTop: `1px solid ${C.border}`,
                            borderBottom: `1px solid ${C.border}`,
                            boxSizing: 'border-box',
                            fontFamily: 'Arial',
                        }}>
                            {/* VALVES header */}
                            <div style={{
                                position: 'absolute', left: 0, right: 0, top: Y_HEADER, height: 38,
                                background: '#510424',
                                borderBottom: `1px solid ${C.border}`,
                                display: 'flex', alignItems: 'center', justifyContent: 'center',
                                fontSize: 18, letterSpacing: 3, fontWeight: 'bold',
                            }}>
                                VALVES
                            </div>

                            {/* OXYGEN / KEROSENE subheaders with flanking lines */}
                            <div style={{
                                position: 'absolute', left: 0, right: 0, top: Y_SUBHEAD + 8, height: 22,
                                display: 'flex',
                            }}>
                                {(['OXYGEN', 'KEROSENE'] as const).map((label, idx) => (
                                    <div key={label} style={{
                                        width: CARD_W, marginLeft: idx === 0 ? OUTER_X : GAP_X,
                                        display: 'flex', alignItems: 'center', gap: 6,
                                    }}>
                                        <div style={{ flex: 1, height: 2, background: '#fff' }} />
                                        <div style={{ fontSize: 13, letterSpacing: 1 }}>{label}</div>
                                        <div style={{ flex: 1, height: 2, background: '#fff' }} />
                                    </div>
                                ))}
                            </div>

                            {/* Oxidiser / fuel row */}
                            <div style={{ position: 'absolute', top: Y_ROW_PBV, left: leftX }}>
                                <ValveToggle name="PBV-101" width={CARD_W} height={CARD_H} />
                            </div>
                            <div style={{ position: 'absolute', top: Y_ROW_PBV, left: rightX }}>
                                <ValveToggle name="PBV-201" width={CARD_W} height={CARD_H} />
                            </div>

                            {/* NITROGEN divider with flanking lines */}
                            <div style={{
                                position: 'absolute', left: 0, right: 0, top: Y_NITRO, height: 22,
                                display: 'flex', alignItems: 'center',
                                padding: `0 ${OUTER_X}px`,
                            }}>
                                <div style={{ flex: 1, height: 2, background: '#fff' }} />
                                <div style={{ fontSize: 14, letterSpacing: 1.5, padding: '0 14px' }}>NITROGEN</div>
                                <div style={{ flex: 1, height: 2, background: '#fff' }} />
                            </div>

                            {/* Nitrogen rows (5) */}
                            {[
                                [Y_ROW_SV01, 'SV-001', 'SV-002'],
                                [Y_ROW_SV03, 'SV-003', 'SV-004'],
                                [Y_ROW_SV05, 'SV-005', 'SV-006'],
                                [Y_ROW_SVG01, 'SVG-001', 'SVG-002'],
                                [Y_ROW_SVG03, 'SVG-003', 'BVG-101'],
                                [Y_ROW_IGN, 'IGN-002', 'ALM-001'],
                            ].map(([y, l, r]) => (
                                <React.Fragment key={l as string}>
                                    <div style={{ position: 'absolute', top: y as number, left: leftX }}>
                                        <ValveToggle name={l as string} width={CARD_W} height={CARD_H} />
                                    </div>
                                    <div style={{ position: 'absolute', top: y as number, left: rightX }}>
                                        <ValveToggle name={r as string} width={CARD_W} height={CARD_H} />
                                    </div>
                                </React.Fragment>
                            ))}

                            {/* ACTUATE */}
                            <button
                                onClick={actuate}
                                style={{
                                    position: 'absolute', top: Y_ACTUATE, left: OUTER_X,
                                    width: SIDEBAR_W - OUTER_X * 2, height: 68,
                                    background: '#b7500a',
                                    border: `1px solid ${C.border}`,
                                    borderRadius: 0,
                                    color: C.text,
                                    fontSize: 26, fontWeight: 'bold', letterSpacing: 3,
                                    cursor: 'pointer', fontFamily: 'Arial',
                                }}
                                onMouseDown={e => (e.currentTarget.style.backgroundColor = '#865006')}
                                onMouseUp={e => (e.currentTarget.style.backgroundColor = '#b7500a')}
                                onMouseLeave={e => (e.currentTarget.style.backgroundColor = '#b7500a')}
                            >
                                ACTUATE
                            </button>

                            {/* IGNITION SEQUENCE header */}
                            <div style={{
                                position: 'absolute', left: 0, right: 0, top: Y_IGN_HEADER, height: 36,
                                background: '#35713e',
                                borderTop: `1px solid ${C.border}`,
                                borderBottom: `1px solid ${C.border}`,
                                display: 'flex', alignItems: 'center', justifyContent: 'center',
                                fontSize: 20, fontWeight: 'bold', letterSpacing: 1.5,
                            }}>
                                IGNITION SEQUENCE:
                            </div>

                            {/* UPDATE SEQUENCES */}
                            <button
                                onClick={refreshSequences}
                                style={{
                                    position: 'absolute', top: Y_UPDATE, left: OUTER_X,
                                    width: SIDEBAR_W - OUTER_X * 2, height: 58,
                                    background: '#808080',
                                    border: `1px solid ${C.border}`,
                                    borderRadius: 0,
                                    color: C.text,
                                    fontSize: 22, fontWeight: 'bold', letterSpacing: 1.5,
                                    cursor: 'pointer', fontFamily: 'Arial',
                                }}
                                onMouseDown={e => (e.currentTarget.style.backgroundColor = '#5e5e5e')}
                                onMouseUp={e => (e.currentTarget.style.backgroundColor = '#808080')}
                                onMouseLeave={e => (e.currentTarget.style.backgroundColor = '#808080')}
                            >
                                UPDATE SEQUENCES
                            </button>

                            {/* Sequence dropdown */}
                            <select
                                value={selectedSequence}
                                onChange={(e) => setSelectedSequence(e.target.value)}
                                style={{
                                    position: 'absolute', top: Y_SEL, left: '50%',
                                    transform: 'translateX(-50%)',
                                    width: 220, height: 38,
                                    background: '#000', color: C.text,
                                    border: `1px solid ${C.border}`,
                                    fontSize: 20, fontFamily: 'Arial',
                                    textAlign: 'center', textAlignLast: 'center',
                                    cursor: 'pointer',
                                }}
                            >
                                {availableSequences.map(s => (
                                    <option key={s.id} value={s.id}>{s.name}</option>
                                ))}
                            </select>

                            {/* START / IGNITION button */}
                            <button
                                onClick={() => {
                                    const seq = availableSequences.find(s => s.id === selectedSequence);
                                    startCountdown(
                                        selectedSequence,
                                        seq?.tMin ?? -6,
                                        seq?.tMax ?? 9,
                                    );
                                }}
                                style={{
                                    position: 'absolute', top: Y_START, left: OUTER_X,
                                    width: SIDEBAR_W - OUTER_X * 2, height: 64,
                                    background: '#cb2a2a',
                                    border: `1px solid ${C.border}`,
                                    borderRadius: 0,
                                    color: C.text,
                                    fontSize: selectedSequence.length > 10 ? 22 : 28,
                                    fontWeight: 'bold', letterSpacing: 2,
                                    cursor: 'pointer', fontFamily: 'Arial',
                                    textTransform: 'uppercase',
                                }}
                                onMouseDown={e => (e.currentTarget.style.backgroundColor = '#8c1f1f')}
                                onMouseUp={e => (e.currentTarget.style.backgroundColor = '#cb2a2a')}
                                onMouseLeave={e => (e.currentTarget.style.backgroundColor = '#cb2a2a')}
                            >
                                {selectedSequence.toUpperCase()}
                            </button>

                            {/* STOP SEQUENCE */}
                            <button
                                onClick={stopSequence}
                                style={{
                                    position: 'absolute', top: Y_STOP, left: OUTER_X,
                                    width: SIDEBAR_W - OUTER_X * 2, height: 56,
                                    background: '#FF8800',
                                    border: `1px solid ${C.border}`,
                                    borderRadius: 0,
                                    color: C.text,
                                    fontSize: 22, fontWeight: 'bold', letterSpacing: 2,
                                    cursor: 'pointer', fontFamily: 'Arial',
                                    textTransform: 'uppercase',
                                }}
                                onMouseDown={e => (e.currentTarget.style.backgroundColor = '#b25d00')}
                                onMouseUp={e => (e.currentTarget.style.backgroundColor = '#FF8800')}
                                onMouseLeave={e => (e.currentTarget.style.backgroundColor = '#FF8800')}
                            >
                                STOP SEQUENCE
                            </button>
                        </div>
                    );
                })()}

                {/*
                    ================ BOTTOM STATUS BLOCKS ================
                    Two equal-width columns at the same baseline:
                      - LOAD CELL on the left (header + 4 readouts)
                      - COUNTDOWN (header + TIMER) on the right, with
                        ABORT flush beneath it so both columns end at
                        exactly the same Y.
                */}
                {(() => {
                    const Y_TOP = 1145;
                    const Y_BOTTOM = 1370;
                    const TOTAL_H = Y_BOTTOM - Y_TOP;      // 225
                    const W = 440;
                    const LEFT_X = 1120;
                    const GAP = 20;
                    const RIGHT_X = LEFT_X + W + GAP;
                    const HEADER_H = 38;
                    const ABORT_H = 90;
                    return (
                        <>
                            {/* LOAD CELL */}
                            <div style={{
                                position: 'absolute', left: LEFT_X, top: Y_TOP,
                                width: W, height: TOTAL_H,
                                background: '#000',
                                border: `1px solid ${C.border}`,
                                boxSizing: 'border-box',
                            }}>
                                <div style={{
                                    width: '100%', height: HEADER_H,
                                    background: '#0a3a7f',
                                    borderBottom: `1px solid ${C.border}`,
                                    display: 'flex', alignItems: 'center', justifyContent: 'center',
                                    fontSize: 20, fontWeight: 'bold', letterSpacing: 2,
                                }}>
                                    LOAD CELL
                                </div>
                                <div style={{
                                    padding: '16px 28px', fontSize: 18,
                                    fontFamily: 'Arial, sans-serif',
                                }}>
                                    {['LC001', 'LC002', 'LC003', 'LC004'].map(n => (
                                        <div key={n} style={{
                                            display: 'flex', marginBottom: 10, alignItems: 'baseline',
                                        }}>
                                            <span style={{ color: C.text, width: 80 }}>{n}:</span>
                                            <span style={{
                                                color: C.readout, flex: 1, textAlign: 'right',
                                                fontFamily: 'Consolas, monospace',
                                            }}>{lc(n)}</span>
                                            <span style={{ color: C.unit, marginLeft: 10, width: 30 }}>lbf</span>
                                        </div>
                                    ))}
                                </div>
                            </div>

                            {/* COUNTDOWN (header + TIMER) */}
                            <div style={{
                                position: 'absolute', left: RIGHT_X, top: Y_TOP,
                                width: W, height: TOTAL_H - ABORT_H - 2,
                                background: '#000',
                                border: `1px solid ${C.border}`,
                                boxSizing: 'border-box',
                            }}>
                                <div style={{
                                    width: '100%', height: HEADER_H,
                                    background: '#398778',
                                    borderBottom: `1px solid ${C.border}`,
                                    display: 'flex', alignItems: 'center', justifyContent: 'center',
                                    fontSize: 20, fontWeight: 'bold', letterSpacing: 2,
                                }}>
                                    COUNTDOWN
                                </div>
                                <div style={{
                                    height: TOTAL_H - ABORT_H - 2 - HEADER_H,
                                    display: 'flex', alignItems: 'center', justifyContent: 'center',
                                    fontSize: 46, fontWeight: 'bold', letterSpacing: 2,
                                    fontFamily: 'Arial',
                                }}>
                                    TIMER:&nbsp;
                                    <span style={{ fontFamily: 'Consolas, monospace' }}>
                                        {timer.toFixed(1)}
                                    </span>
                                </div>
                            </div>

                            {/* ABORT button flush under COUNTDOWN */}
                            <button
                                onClick={abort}
                                style={{
                                    position: 'absolute',
                                    left: RIGHT_X,
                                    top: Y_TOP + TOTAL_H - ABORT_H,
                                    width: W, height: ABORT_H,
                                    background: '#FF0000',
                                    border: `1px solid ${C.border}`,
                                    borderRadius: 0,
                                    color: C.text,
                                    fontSize: 34, fontWeight: 'bold', letterSpacing: 4,
                                    cursor: 'pointer', fontFamily: 'Arial',
                                }}
                                onMouseDown={e => (e.currentTarget.style.backgroundColor = '#8c1818')}
                                onMouseUp={e => (e.currentTarget.style.backgroundColor = '#FF0000')}
                                onMouseLeave={e => (e.currentTarget.style.backgroundColor = '#FF0000')}
                            >
                                ABORT
                            </button>
                        </>
                    );
                })()}

                {/*
                    ================ BOTTOM-LEFT STATUS AREA ================
                    Tight stack matching the reference:
                      - FIT / FULL (tiny inline buttons)
                      - Close Server (red flat button)
                      - SERVER STATUS:  <NOT CONNECTED | CONNECTED>
                */}
                <div style={{
                    position: 'absolute', left: 18, top: 1212,
                    display: 'flex', gap: 4, fontFamily: 'Arial',
                }}>
                    <button
                        onClick={() => setScale(Math.min(
                            (wrapperRef.current?.clientWidth ?? BASE_W) / BASE_W,
                            (wrapperRef.current?.clientHeight ?? BASE_H) / BASE_H,
                        ))}
                        style={{ padding: '2px 10px', fontSize: 12, cursor: 'pointer', fontFamily: 'Arial' }}
                    >FIT</button>
                    <button
                        onClick={() => setScale(1)}
                        style={{ padding: '2px 10px', fontSize: 12, cursor: 'pointer', fontFamily: 'Arial' }}
                    >FULL</button>
                </div>

                <button
                    style={{
                        position: 'absolute', left: 18, top: 1246,
                        width: 210, height: 46,
                        background: '#941010',
                        border: `1px solid ${C.border}`,
                        borderRadius: 0,
                        color: C.text,
                        fontSize: 20, fontWeight: 'bold', letterSpacing: 1,
                        cursor: 'pointer', fontFamily: 'Arial',
                    }}
                    onMouseDown={e => (e.currentTarget.style.backgroundColor = '#5d0a0a')}
                    onMouseUp={e => (e.currentTarget.style.backgroundColor = '#941010')}
                    onMouseLeave={e => (e.currentTarget.style.backgroundColor = '#941010')}
                >
                    Close Server
                </button>

                <div style={{
                    position: 'absolute', left: 18, top: 1320,
                    display: 'flex', alignItems: 'baseline', gap: 40,
                    fontSize: 22, fontFamily: 'Arial', letterSpacing: 1,
                }}>
                    <span>SERVER STATUS:</span>
                    <span style={{
                        color: serverStatus ? '#32CD32' : '#ffffff',
                        fontWeight: 'bold',
                    }}>
                        {serverStatus ? 'CONNECTED' : 'NOT CONNECTED'}
                    </span>
                </div>
            </div>
        </div>
    );
};

export default Dashboard;
