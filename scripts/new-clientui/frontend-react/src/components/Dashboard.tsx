import React, { useRef, useState, useEffect, useCallback } from 'react';
import { useBackend } from '../context/BackendContext';
import { Gage } from './Gage';
import { Valve } from './Valve';
import { ValveState } from './ValveState';
import { ValveToggle } from './ValveToggle';

const BASE_W = 2560;
const BASE_H = 1440;

const AVAILABLE_SEQUENCES = [
    { id: 'ignition', name: 'ignition', startTime: 6 },
    { id: 'vent', name: 'vent', startTime: 0 },
    { id: 'purge', name: 'purge', startTime: 2 },
];

const Dashboard: React.FC = () => {
    const { gauges, timer, serverStatus, abort, actuate, startCountdown } = useBackend();
    const wrapperRef = useRef<HTMLDivElement>(null);
    const [scale, setScale] = useState(1);
    const [selectedSequence, setSelectedSequence] = useState(AVAILABLE_SEQUENCES[0].id);

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

    const lc = (name: string) => gauges[name]?.toFixed(1) ?? 'N/A';

    return (
        <div
            ref={wrapperRef}
            style={{
                width: '100vw',
                height: '100vh',
                backgroundColor: '#000000',
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
                    top: 0,
                    left: 0,
                    fontFamily: 'Arial, sans-serif',
                    color: '#ffffff',
                }}
            >
                {/* ================================================ */}
                {/* LOGO */}
                {/* ================================================ */}
                <img
                    src="/assets/logo white.png"
                    alt="Logo"
                    style={{ position: 'absolute', left: 57, top: 34, width: 281, height: 102, objectFit: 'contain' }}
                />

                {/* ================================================ */}
                {/* TITLE */}
                {/* ================================================ */}
                <div style={{
                    position: 'absolute', left: 332, top: 57,
                    fontSize: 50, fontWeight: 'normal', whiteSpace: 'nowrap',
                }}>
                    MISSION CONTROL - ATLAS
                </div>

                {/* ================================================ */}
                {/* BACKGROUND SCHEMATIC SVG */}
                {/* ================================================ */}
                <img
                    src="/assets/ATLAS_GUI_FALL_2025.svg"
                    alt="Schematic"
                    style={{
                        position: 'absolute', left: 0, top: 122,
                        width: 2035, height: 1236,
                        objectFit: 'contain',
                    }}
                />

                {/* ================================================ */}
                {/* COMPRESSOR OVERLAY */}
                {/* ================================================ */}
                <img
                    src="/assets/DIAGRAM_Compressor.svg"
                    alt="Compressor"
                    style={{
                        position: 'absolute', left: 197, top: 207,
                        width: 620, height: 181,
                        objectFit: 'contain',
                    }}
                />

                {/* ================================================ */}
                {/* GAGE BOXES ON THE SCHEMATIC */}
                {/* ================================================ */}

                {/* PT401 - next to compressor */}
                <GageBox x={57} y={312} w={179} h={30}>
                    <Gage name="PT401" x={0} y={0} />
                </GageBox>

                {/* PT001 - Nitrogen */}
                <GageBox x={150} y={822} w={180} h={30}>
                    <Gage name="PT001" x={0} y={0} />
                </GageBox>

                {/* PT002 - Nitrogen reg A */}
                <GageBox x={361} y={429} w={180} h={30}>
                    <Gage name="PT002" x={0} y={0} />
                </GageBox>

                {/* PT003 - Nitrogen reg B */}
                <GageBox x={581} y={429} w={180} h={30}>
                    <Gage name="PT003" x={0} y={0} />
                </GageBox>

                {/* PT004 - Mid manifold  */}
                <GageBox x={1102} y={457} w={198} h={30}>
                    <Gage name="PT004" x={0} y={0} />
                </GageBox>

                {/* PT005 + TC001 */}
                <GageBox x={584} y={818} w={180} h={68}>
                    <Gage name="PT005" x={0} y={0} />
                    <Gage name="TC001" x={0} y={32} unit="C" />
                </GageBox>

                {/* PT006 + TC002 */}
                <GageBox x={1102} y={834} w={198} h={72}>
                    <Gage name="PT006" x={0} y={0} />
                    <Gage name="TC002" x={0} y={32} unit="C" />
                </GageBox>

                {/* PT201 + TC201 + FM201 - Fuel tank */}
                <GageBox x={1419} y={396} w={180} h={93}>
                    <Gage name="PT201" x={0} y={0} />
                    <Gage name="TC201" x={0} y={32} unit="C" />
                    <Gage name="FM201" x={0} y={64} unit="GPM" />
                </GageBox>

                {/* PT101 + TC101 + FM101 - LOX tank */}
                <GageBox x={1451} y={803} w={180} h={93}>
                    <Gage name="PT101" x={0} y={0} />
                    <Gage name="TC101" x={0} y={32} unit="C" />
                    <Gage name="FM101" x={0} y={64} unit="GPM" />
                </GageBox>

                {/* PT202 + TC202 - Fuel injector  */}
                <GageBox x={1674} y={438} w={180} h={62}>
                    <Gage name="PT202" x={0} y={0} />
                    <Gage name="TC202" x={0} y={32} unit="C" />
                </GageBox>

                {/* PT102 + TC102 - LOX injector */}
                <GageBox x={1716} y={811} w={180} h={62}>
                    <Gage name="PT102" x={0} y={0} />
                    <Gage name="TC102" x={0} y={32} unit="C" />
                </GageBox>

                {/* PTF401 - Engine PT top */}
                <GageBox x={1872} y={460} w={173} h={30} transparent>
                    <Gage name="PTF401" x={0} y={0} />
                </GageBox>

                {/* PTO401 + TC103 - Engine PT left */}
                <GageBox x={1664} y={588} w={180} h={60}>
                    <Gage name="PTO401" x={0} y={0} />
                    <Gage name="TC103" x={0} y={32} unit="C" />
                </GageBox>

                {/* Engine PT cover (black rect) */}
                <div style={{
                    position: 'absolute', left: 1872, top: 652,
                    width: 150, height: 60,
                    backgroundColor: '#000000',
                }} />

                {/* PT301 - Purge */}
                <GageBox x={230} y={1190} w={180} h={30}>
                    <Gage name="PT301" x={0} y={0} />
                </GageBox>

                {/* ================================================ */}
                {/* VALVE COLOR OVERLAYS */}
                {/* ================================================ */}
                <Valve name="PBV-001" x={404} y={596} width={67} height={29} />
                <Valve name="PBV-002" x={321} y={685} width={67} height={29} />
                <Valve name="PBV-003" x={627} y={514} width={67} height={29} />
                <Valve name="PBV-004" x={629} y={757} width={67} height={29} />
                <Valve name="PBV-005" x={838} y={464} width={67} height={29} />
                <Valve name="PBV-006" x={860} y={836} width={67} height={29} />
                <Valve name="PBV-101" x={1509} y={755} width={67} height={29} />
                <Valve name="PBV-201" x={1515} y={514} width={67} height={29} />
                <Valve name="PBV-301" x={528} y={1056} width={54} height={30} />
                <Valve name="PBV-302" x={528} y={1197} width={54} height={30} />

                {/* ================================================ */}
                {/* VALVE STATE LABELS */}
                {/* ================================================ */}
                <ValveState name="PBV-001" x={394} y={643} />
                <ValveState name="PBV-002" x={317} y={730} />
                <ValveState name="PBV-003" x={619} y={557} />
                <ValveState name="PBV-004" x={619} y={688} />
                <ValveState name="PBV-005" x={829} y={388} />
                <ValveState name="PBV-006" x={845} y={884} />
                <ValveState name="PBV-201" x={1510} y={562} />
                <ValveState name="PBV-101" x={1503} y={689} />
                <ValveState name="PBV-301" x={515} y={988} />
                <ValveState name="PBV-302" x={515} y={1239} />

                {/* ================================================ */}
                {/* RIGHT PANEL: VALVES CONTROLS */}
                {/* ================================================ */}
                <div style={{
                    position: 'absolute', left: 2061, top: 0,
                    width: 499, height: 931,
                    backgroundColor: '#000000',
                    border: '1px solid #ffffff',
                    boxSizing: 'border-box',
                }}>
                    {/* Header */}
                    <div style={{
                        width: '100%', height: 57,
                        backgroundColor: '#510424',
                        border: '1px solid #ffffff',
                        display: 'flex', alignItems: 'center', justifyContent: 'center',
                        fontSize: 25, boxSizing: 'border-box',
                    }}>
                        VALVES
                    </div>

                    {/* Section: OXYGEN / KEROSENE labels */}
                    <div style={{ position: 'absolute', top: 73, width: '100%' }}>
                        <div style={{ display: 'flex', justifyContent: 'space-around', padding: '0 20px' }}>
                            <span style={{ fontSize: 20 }}>OXYGEN</span>
                            <span style={{ fontSize: 20 }}>KEROSENE</span>
                        </div>
                        {/* Divider lines */}
                        <div style={{ display: 'flex', justifyContent: 'space-between', padding: '0 20px', marginTop: 5 }}>
                            <div style={{ width: 59, height: 2, backgroundColor: '#ffffff' }} />
                            <div style={{ width: 60, height: 2, backgroundColor: '#ffffff' }} />
                            <div style={{ width: 50, height: 2, backgroundColor: '#ffffff' }} />
                            <div style={{ width: 50, height: 2, backgroundColor: '#ffffff' }} />
                        </div>
                    </div>

                    {/* Valve Toggles Row 1: PBV-101 / PBV-201 */}
                    <div style={{ position: 'absolute', top: 112, left: 20 }}><ValveToggle name="PBV-101" /></div>
                    <div style={{ position: 'absolute', top: 112, right: 20 }}><ValveToggle name="PBV-201" /></div>

                    {/* NITROGEN section divider */}
                    <div style={{ position: 'absolute', top: 286, width: '100%', textAlign: 'center', fontSize: 20 }}>
                        NITROGEN
                    </div>
                    <div style={{ position: 'absolute', top: 297, left: 20, width: 167, height: 2, backgroundColor: '#ffffff' }} />
                    <div style={{ position: 'absolute', top: 297, right: 20, width: 167, height: 2, backgroundColor: '#ffffff' }} />

                    {/* Row 2: PBV-301 / PBV-302 */}
                    <div style={{ position: 'absolute', top: 339, left: 20 }}><ValveToggle name="PBV-301" /></div>
                    <div style={{ position: 'absolute', top: 339, right: 20 }}><ValveToggle name="PBV-302" /></div>

                    {/* Row 3: PBV-001 / PBV-002 */}
                    <div style={{ position: 'absolute', top: 424, left: 20 }}><ValveToggle name="PBV-001" /></div>
                    <div style={{ position: 'absolute', top: 424, right: 20 }}><ValveToggle name="PBV-002" /></div>

                    {/* Row 4: PBV-003 / PBV-004 */}
                    <div style={{ position: 'absolute', top: 513, left: 20 }}><ValveToggle name="PBV-003" /></div>
                    <div style={{ position: 'absolute', top: 513, right: 20 }}><ValveToggle name="PBV-004" /></div>

                    {/* Row 5: PBV-005 / PBV-006 */}
                    <div style={{ position: 'absolute', top: 603, left: 20 }}><ValveToggle name="PBV-005" /></div>
                    <div style={{ position: 'absolute', top: 603, right: 20 }}><ValveToggle name="PBV-006" /></div>

                    {/* Row 6: IGN-002 / ALM-001 */}
                    <div style={{ position: 'absolute', top: 693, left: 20 }}><ValveToggle name="IGN-002" /></div>
                    <div style={{ position: 'absolute', top: 693, right: 20 }}><ValveToggle name="ALM-001" /></div>

                    {/* ACTUATE button */}
                    <button
                        onClick={actuate}
                        style={{
                            position: 'absolute', top: 815, left: '50%', transform: 'translateX(-50%)',
                            width: 400, height: 77,
                            backgroundColor: '#b7500a',
                            border: '1px solid #ffffff',
                            borderRadius: 4,
                            color: '#ffffff',
                            fontSize: 31, fontWeight: 'bold',
                            cursor: 'pointer',
                            fontFamily: 'Arial',
                        }}
                        onMouseDown={e => (e.currentTarget.style.backgroundColor = '#865006')}
                        onMouseUp={e => (e.currentTarget.style.backgroundColor = '#b7500a')}
                        onMouseLeave={e => (e.currentTarget.style.backgroundColor = '#b7500a')}
                    >
                        ACTUATE
                    </button>
                </div>

                {/* ================================================ */}
                {/* RIGHT PANEL: IGNITION SEQUENCE */}
                {/* ================================================ */}
                <div style={{
                    position: 'absolute', left: 2061, top: 930,
                    width: 499, height: 390,
                    backgroundColor: '#000000',
                    border: '1px solid #ffffff',
                    boxSizing: 'border-box',
                }}>
                    {/* Header */}
                    <div style={{
                        width: '100%', height: 47,
                        backgroundColor: '#35713e',
                        border: '1px solid #ffffff',
                        display: 'flex', alignItems: 'center', justifyContent: 'center',
                        fontSize: 29, fontWeight: 'bold',
                        boxSizing: 'border-box',
                    }}>
                        IGNITION SEQUENCE:
                    </div>

                    {/* UPDATE SEQUENCES button */}
                    <button
                        style={{
                            position: 'absolute', left: 32, top: 65,
                            width: 435, height: 64,
                            backgroundColor: '#808080',
                            border: '1px solid #ffffff',
                            borderRadius: 4,
                            color: '#ffffff',
                            fontSize: 30, fontWeight: 'bold',
                            cursor: 'pointer',
                            fontFamily: 'Arial',
                        }}
                        onMouseDown={e => (e.currentTarget.style.backgroundColor = '#732727')}
                        onMouseUp={e => (e.currentTarget.style.backgroundColor = '#808080')}
                        onMouseLeave={e => (e.currentTarget.style.backgroundColor = '#808080')}
                    >
                        UPDATE SEQUENCES
                    </button>

                    {/* Sequence name field */}
                    <select
                        value={selectedSequence}
                        onChange={(e) => setSelectedSequence(e.target.value)}
                        style={{
                            position: 'absolute', left: 128, top: 170,
                            width: 242, height: 43,
                            backgroundColor: '#000000',
                            color: '#ffffff',
                            border: '1px solid #ffffff',
                            fontSize: 30, fontFamily: 'Arial',
                            textAlign: 'center',
                            cursor: 'pointer',
                        }}
                    >
                        {AVAILABLE_SEQUENCES.map(seq => (
                            <option key={seq.id} value={seq.id}>{seq.name}</option>
                        ))}
                    </select>

                    {/* START SEQUENCE button */}
                    <button
                        onClick={() => {
                            const seq = AVAILABLE_SEQUENCES.find(s => s.id === selectedSequence);
                            startCountdown(selectedSequence, seq?.startTime || 0);
                        }}
                        style={{
                            position: 'absolute', left: 32, top: 316,
                            width: 435, height: 64,
                            backgroundColor: '#cb2a2a',
                            border: '1px solid #ffffff',
                            borderRadius: 4,
                            color: '#ffffff',
                            fontSize: selectedSequence.length > 10 ? 25 : 35,
                            fontWeight: 'bold',
                            cursor: 'pointer',
                            fontFamily: 'Arial',
                            textTransform: 'uppercase',
                        }}
                        onMouseDown={e => (e.currentTarget.style.backgroundColor = '#732727')}
                        onMouseUp={e => (e.currentTarget.style.backgroundColor = '#cb2a2a')}
                        onMouseLeave={e => (e.currentTarget.style.backgroundColor = '#cb2a2a')}
                    >
                        {selectedSequence}
                    </button>
                </div>

                {/* ================================================ */}
                {/* COUNTDOWN BOX */}
                {/* ================================================ */}
                <div style={{
                    position: 'absolute', left: 1576, top: 1122,
                    width: 449, height: 247,
                    backgroundColor: '#000000',
                    border: '1px solid #ffffff',
                    boxSizing: 'border-box',
                }}>
                    <div style={{
                        width: '100%', height: 39,
                        backgroundColor: '#398778',
                        border: '1px solid #ffffff',
                        display: 'flex', alignItems: 'center', justifyContent: 'center',
                        fontSize: 30, fontWeight: 'bold',
                        boxSizing: 'border-box',
                    }}>
                        COUNTDOWN
                    </div>
                    <div style={{
                        display: 'flex', alignItems: 'center', justifyContent: 'center',
                        height: 208, fontSize: 55, fontWeight: 'bold',
                    }}>
                        TIMER: {timer.toFixed(1)}
                    </div>
                </div>

                {/* ================================================ */}
                {/* ABORT BUTTON */}
                {/* ================================================ */}
                <button
                    onClick={abort}
                    style={{
                        position: 'absolute', left: 1585, top: 1272,
                        width: 432, height: 72,
                        backgroundColor: '#FF0000',
                        border: '1px solid #ffffff',
                        borderRadius: 4,
                        color: '#ffffff',
                        fontSize: 30, fontWeight: 'bold',
                        cursor: 'pointer',
                        fontFamily: 'Arial',
                    }}
                    onMouseDown={e => (e.currentTarget.style.backgroundColor = '#732727')}
                    onMouseUp={e => (e.currentTarget.style.backgroundColor = '#FF0000')}
                    onMouseLeave={e => (e.currentTarget.style.backgroundColor = '#FF0000')}
                >
                    ABORT
                </button>

                {/* ================================================ */}
                {/* LOAD CELL BOX */}
                {/* ================================================ */}
                <div style={{
                    position: 'absolute', left: 1124, top: 1123,
                    width: 453, height: 247,
                    backgroundColor: '#000000',
                    border: '1px solid #ffffff',
                    boxSizing: 'border-box',
                }}>
                    <div style={{
                        width: '100%', height: 45,
                        backgroundColor: '#0a3a7f',
                        border: '1px solid #ffffff',
                        display: 'flex', alignItems: 'center', justifyContent: 'center',
                        fontSize: 25, fontWeight: 'bold',
                        boxSizing: 'border-box',
                    }}>
                        LOAD CELL
                    </div>
                    <div style={{ padding: '10px 20px', fontFamily: 'Arial', fontSize: 17 }}>
                        <div style={{ display: 'flex', marginBottom: 6 }}>
                            <span style={{ color: '#ffffff', width: 80 }}>LC001:</span>
                            <span style={{ color: '#2ad12f', width: 80, textAlign: 'right' }}>{lc('LC001')}</span>
                            <span style={{ color: '#69eef0', marginLeft: 10 }}>lbf</span>
                        </div>
                        <div style={{ display: 'flex', marginBottom: 6 }}>
                            <span style={{ color: '#ffffff', width: 80 }}>LC002:</span>
                            <span style={{ color: '#2ad12f', width: 80, textAlign: 'right' }}>{lc('LC002')}</span>
                            <span style={{ color: '#69eef0', marginLeft: 10 }}>lbf</span>
                        </div>
                        <div style={{ display: 'flex', marginBottom: 6 }}>
                            <span style={{ color: '#ffffff', width: 80 }}>LC003:</span>
                            <span style={{ color: '#2ad12f', width: 80, textAlign: 'right' }}>{lc('LC003')}</span>
                            <span style={{ color: '#69eef0', marginLeft: 10 }}>lbf</span>
                        </div>
                        <div style={{ display: 'flex' }}>
                            <span style={{ color: '#ffffff', width: 80 }}>LC004:</span>
                            <span style={{ color: '#2ad12f', width: 80, textAlign: 'right' }}>{lc('LC004')}</span>
                            <span style={{ color: '#69eef0', marginLeft: 10 }}>lbf</span>
                        </div>
                    </div>
                </div>

                {/* ================================================ */}
                {/* FIT / FULL buttons */}
                {/* ================================================ */}
                <div style={{ position: 'absolute', left: 9, top: 1241, display: 'flex', gap: 4 }}>
                    <button
                        onClick={() => setScale(Math.min(
                            (wrapperRef.current?.clientWidth ?? BASE_W) / BASE_W,
                            (wrapperRef.current?.clientHeight ?? BASE_H) / BASE_H
                        ))}
                        style={{ padding: '4px 12px', fontSize: 14, cursor: 'pointer', fontFamily: 'Arial' }}
                    >
                        FIT
                    </button>
                    <button
                        onClick={() => setScale(1)}
                        style={{ padding: '4px 12px', fontSize: 14, cursor: 'pointer', fontFamily: 'Arial' }}
                    >
                        FULL
                    </button>
                </div>

                {/* CLOSE SERVER button */}
                <button
                    style={{
                        position: 'absolute', left: 9, top: 1299,
                        width: 237, height: 58,
                        backgroundColor: '#941010',
                        border: '1px solid #ffffff',
                        color: '#ffffff',
                        fontSize: 25, fontWeight: 'bold',
                        cursor: 'pointer',
                        fontFamily: 'Arial',
                    }}
                    onMouseDown={e => (e.currentTarget.style.backgroundColor = '#732727')}
                    onMouseUp={e => (e.currentTarget.style.backgroundColor = '#941010')}
                    onMouseLeave={e => (e.currentTarget.style.backgroundColor = '#941010')}
                >
                    Close Server
                </button>

                {/* ================================================ */}
                {/* SERVER STATUS BAR */}
                {/* ================================================ */}
                <div style={{
                    position: 'absolute', left: 0, top: 1380,
                    width: 2000, height: 44,
                    display: 'flex', alignItems: 'center',
                    fontSize: 30,
                }}>
                    <span>SERVER STATUS: </span>
                    <span style={{
                        color: serverStatus ? '#32CD32' : '#ffffff',
                        marginLeft: 50,
                    }}>
                        {serverStatus ? 'CONNECTED' : 'NOT CONNECTED'}
                    </span>
                </div>
            </div>
        </div>
    );
};

/* ========================================== */
/* Helper: Black box with white border for gages */
/* ========================================== */
const GageBox: React.FC<{
    x: number; y: number; w: number; h: number;
    transparent?: boolean;
    children: React.ReactNode;
}> = ({ x, y, w, h, transparent, children }) => (
    <div style={{
        position: 'absolute', left: x, top: y,
        width: w, height: h,
        backgroundColor: transparent ? 'transparent' : '#000000',
        border: '1px solid #ffffff',
        boxSizing: 'border-box',
    }}>
        {children}
    </div>
);

export default Dashboard;
