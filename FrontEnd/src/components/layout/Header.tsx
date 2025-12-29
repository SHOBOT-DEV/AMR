import React, { useState, useEffect } from "react";
import {
    FaBatteryHalf,
    FaPlay,
    FaPause,
    FaExpand,
    FaCompress,
    FaLock,
    FaUnlock,
    FaSignOutAlt,
} from "react-icons/fa";
import { toast } from "react-hot-toast";

type Cell = { id: string; voltage: number | string };
type BatteryStatus = {
    packVoltage: number;
    packCurrent: number;
    stateOfCharge: number;
    temperature: string;
    cycles: number;
    health: string;
    cells?: Cell[];
};
type BridgeStatus = { connected: boolean; error?: string; endpoint?: string };

type Props = {
    bridgeStatus?: BridgeStatus;
    startBridgeConnection?: () => void;
    handleEmergencyToggle?: (locked: boolean) => void;
    handleLogout?: () => void;
    batteryStatus?: BatteryStatus;
    layoutRef?: React.RefObject<HTMLElement | null>;
    onLockChange?: (locked: boolean) => void;
};

const Header: React.FC<Props> = ({
    bridgeStatus = { connected: false, error: "", endpoint: "" },
    startBridgeConnection = () => { },
    handleEmergencyToggle = undefined,
    handleLogout = () => { },
    batteryStatus = {
        packVoltage: 46.9,
        packCurrent: 38.2,
        stateOfCharge: 78,
        temperature: "32°C",
        cycles: 412,
        health: "Good",
        cells: [
            { id: "Cell A", voltage: 3.9 },
            { id: "Cell B", voltage: 3.89 },
            { id: "Cell C", voltage: 3.88 },
            { id: "Cell D", voltage: 3.87 },
        ],
    },
    layoutRef = null,
    onLockChange = undefined,
}) => {
    const [isDropdownOpen, setIsDropdownOpen] = useState(false);
    const [isPlaying, setIsPlaying] = useState(false);

    // user selection + loaders + errors used by right pane actions (kept for compatibility)
    const [selectedUserId, setSelectedUserId] = useState<string | null>(null);
    const [usersLoading, setUsersLoading] = useState(false);
    const [userActionLoading, setUserActionLoading] = useState(false);
    const [usersError, setUsersError] = useState("");

    const [securityPreferences, setSecurityPreferences] = useState({
        twoFactor: true,
        autoLock: true,
        anomalyAlerts: true,
    });

    // Fullscreen
    const [isFullScreen, setIsFullScreen] = useState(false);
    useEffect(() => {
        const onFsChange = () => {
            const fsEl = (document as any).fullscreenElement || (document as any).webkitFullscreenElement;
            setIsFullScreen(fsEl === layoutRef?.current);
        };
        document.addEventListener("fullscreenchange", onFsChange);
        document.addEventListener("webkitfullscreenchange", onFsChange);
        return () => {
            document.removeEventListener("fullscreenchange", onFsChange);
            document.removeEventListener("webkitfullscreenchange", onFsChange);
        };
    }, [layoutRef]);

    const toggleFullScreen = async () => {
        try {
            const target = layoutRef?.current as any;
            if (!target) return;
            const fsEl = (document as any).fullscreenElement || (document as any).webkitFullscreenElement;
            if (!fsEl) {
                if (target.requestFullscreen) await target.requestFullscreen();
                else if (target.webkitRequestFullscreen) target.webkitRequestFullscreen();
            } else {
                if (document.exitFullscreen) await document.exitFullscreen();
                else if ((document as any).webkitExitFullscreen) (document as any).webkitExitFullscreen();
            }
        } catch (e) {
            console.warn("Fullscreen error:", e);
        }
    };

    // Lock
    const LOCK_TOAST_ID = "lock-toast";
    const [isLocked, setIsLocked] = useState(false);
    const handleToggleLock = () => {
        setIsLocked((prev) => {
            const next = !prev;
            toast.dismiss(LOCK_TOAST_ID);
            if (next) {
                toast.error("Console locked", { id: LOCK_TOAST_ID, position: "top-right" });
            } else {
                toast.success("Console unlocked", { id: LOCK_TOAST_ID, position: "top-right" });
            }

            // keep backward compatibility: modify layout root class (MainPage may rely on it)
            try {
                if (layoutRef?.current) {
                    if (next) layoutRef.current.classList.add("is-locked");
                    else layoutRef.current.classList.remove("is-locked");
                }
            } catch { /* ignore */ }

            // notify parent
            try {
                if (typeof onLockChange === "function") onLockChange(next);
            } catch { /* ignore */ }

            return next;
        });
    };

    // Battery modal + battery API
    const [batteryModalOpen, setBatteryModalOpen] = useState(false);
    const [batteryLevel, setBatteryLevel] = useState(100);
    useEffect(() => {
        let batteryObj: any = null;
        let mounted = true;
        const updateLevel = () => {
            if (!batteryObj) return;
            const lvl = Math.round(batteryObj.level * 100);
            if (mounted) setBatteryLevel(lvl);
        };
        if ((navigator as any).getBattery) {
            (navigator as any).getBattery().then((b: any) => {
                batteryObj = b;
                updateLevel();
                batteryObj.addEventListener("levelchange", updateLevel);
            });
        } else {
            setBatteryLevel(100);
        }
        return () => {
            mounted = false;
            if (batteryObj && batteryObj.removeEventListener) batteryObj.removeEventListener("levelchange", updateLevel);
        };
    }, []);

    // Emergency
    const EMERGENCY_TOAST_ID = "emergency-toast";
    const [emergencyClicked, setEmergencyClicked] = useState(false);
    const handleEmergencyToggleLocal = () => {
        setEmergencyClicked((prev) => {
            const next = !prev;
            toast.dismiss();
            if (next) {
                toast.error("Emergency stop engaged", { id: EMERGENCY_TOAST_ID, position: "top-right" });
            } else {
                toast.success("Emergency stop released", { id: EMERGENCY_TOAST_ID, position: "top-right" });
            }
            try {
                if (typeof handleEmergencyToggle === "function") handleEmergencyToggle(next);
            } catch { /* ignore */ }
            return next;
        });
    };

    const batteryClass = (() => {
        if (batteryLevel < 20) return "text-[#ff4d4d]";
        if (batteryLevel < 50) return "text-[#f59e0b]";
        if (batteryLevel > 70) return "text-[#10b981]";
        return "text-[#f97316]";
    })();
    const batteryColor = (() => {
        if (batteryLevel < 20) return "#ff4d4d";
        if (batteryLevel < 50) return "#f59e0b";
        if (batteryLevel > 70) return "#10b981";
        return "#f97316";
    })();

    return (
        <>
            {/* header: apply locked visual state directly when isLocked so it matches previous CSS behavior */}
            <header
                className={`h-[56px] flex items-center gap-3 px-4 fixed top-0 left-0 right-0 z-[1100] ${isLocked ? "pointer-events-none select-none opacity-70 blur-sm" : ""
                    }`}
                style={{ background: "#87ceeb", color: "#fff" }}
            >
                {/* left bridge chip */}
                <div className="flex items-center gap-3">
                    <div
                        className={`inline-flex items-center gap-2 px-2 py-1 rounded-lg min-w-[150px] transition-transform ${bridgeStatus.connected ? "" : "warn-chip"}`}
                        style={{
                            // use exact colors from previous CSS when in warn (offline) state
                            background: bridgeStatus.connected ? "rgba(255,255,255,0.12)" : "rgba(239, 68, 68, 0.16)",
                            color: "#fff",
                            border: bridgeStatus.connected ? "1px solid rgba(16,185,129,0.5)" : "1px solid rgba(239, 68, 68, 0.5)",
                        }}
                        title={
                            bridgeStatus.error
                                ? `Bridge: ${bridgeStatus.error}`
                                : bridgeStatus.connected
                                    ? `Connected to AMR bridge (${bridgeStatus.endpoint})`
                                    : `Bridge offline (${bridgeStatus.endpoint})`
                        }
                    >
                        <span
                            className="rounded-full"
                            style={{
                                width: 10,
                                height: 10,
                                display: "inline-block",
                                boxShadow: "0 0 0 6px rgba(255,255,255,0.08)",
                                background: bridgeStatus.connected ? "#10b981" : "#ef4444",
                            }}
                        />
                        <div className="flex flex-col leading-tight">
                            <span className="text-xs uppercase opacity-90">AMR Bridge</span>
                            <span className="font-bold text-sm">
                                {bridgeStatus.connected ? "Connected" : "Offline"}
                                {bridgeStatus.endpoint ? ` · ${bridgeStatus.endpoint}` : ""}
                            </span>
                        </div>
                        <button
                            type="button"
                            className="ml-2 px-2 py-1 rounded-md text-xs"
                            style={{
                                border: "1px solid rgba(255,255,255,0.35)",
                                background: "rgba(255,255,255,0.12)",
                                color: "#fff",
                            }}
                            onClick={() => {
                                toast.dismiss();
                                toast.success("Retrying bridge connection…", { id: "bridge-retry", position: "top-right", duration: 1500 });
                                startBridgeConnection();
                            }}
                        >
                            Retry
                        </button>
                    </div>
                </div>

                {/* emergency centered */}
                <div className="absolute left-1/2 top-1/2 -translate-x-1/2 -translate-y-1/2 z-10 pointer-events-auto">
                    <button
                        onClick={handleEmergencyToggleLocal}
                        aria-pressed={emergencyClicked}
                        className={`px-3 py-2 rounded-md font-extrabold inline-flex items-center gap-2 ${emergencyClicked ? "" : ""}`}
                        style={
                            emergencyClicked
                                ? { background: "linear-gradient(180deg, #d92b2b, #a10f0f)", color: "#fff" }
                                : { background: "#f1c40f", color: "#111" }
                        }
                    >
                        <span
                            className="inline-flex items-center justify-center rounded-full"
                            style={{
                                width: 14,
                                height: 14,
                                background: emergencyClicked ? "#7a0f0f" : "#d92b2b",
                                display: "inline-flex",
                                alignItems: "center",
                                justifyContent: "center",
                            }}
                        >
                            <span style={{ width: 6, height: 6, borderRadius: 6, background: "#fff", display: "inline-block" }} />
                        </span>
                        <span className="text-sm">Emergency Stop</span>
                    </button>
                </div>

                {/* dropdown */}
                <div className="relative ml-2">
                    <div
                        className="inline-flex items-center gap-2 px-2 py-1 rounded-md cursor-pointer select-none"
                        style={{ background: "rgba(255,255,255,0.08)", color: "#fff", fontWeight: 700, fontSize: 13 }}
                        role="button"
                        tabIndex={0}
                        onClick={() => setIsDropdownOpen((s) => !s)}
                        aria-expanded={isDropdownOpen}
                    >
                        <button
                            onClick={(e) => {
                                e.stopPropagation();
                                setIsPlaying((p) => !p);
                            }}
                            title={isPlaying ? "Pause" : "Play"}
                            className={`w-7 h-7 rounded-md flex items-center justify-center ${isPlaying ? "" : ""}`}
                            style={{ background: isPlaying ? "rgba(255,255,255,0.18)" : "transparent" }}
                            aria-pressed={isPlaying}
                        >
                            {isPlaying ? <FaPause /> : <FaPlay />}
                        </button>
                        <span>Robot: UI EMERGENCY</span>
                        <span className="ml-1">▾</span>
                    </div>

                    {isDropdownOpen && (
                        <div className="absolute top-full left-0 mt-2 min-w-[260px] bg-white text-black rounded-md p-3 shadow-[0_6px_18px_rgba(0,0,0,0.12)] z-50">
                            <div className="font-bold mb-1">Robot: UI EMERGENCY</div>
                            <div className="text-sm">There is no mission running. Initiate a mission to get started.</div>
                        </div>
                    )}
                </div>

                {/* right actions */}
                <div className="ml-auto flex items-center gap-3">
                    <button
                        className={`inline-flex items-center gap-2 bg-transparent p-0`}
                        type="button"
                        onClick={() => setBatteryModalOpen(true)}
                        title="Battery status"
                        aria-label="Battery status"
                    >
                        <FaBatteryHalf className={`text-[18px] ${batteryClass}`} color={batteryColor} />
                        <div className="font-bold text-sm min-w-[36px] text-right" style={{ color: "#fff" }}>{batteryLevel}%</div>
                    </button>

                    <button
                        onClick={toggleFullScreen}
                        title={isFullScreen ? "Exit Fullscreen" : "Fullscreen"}
                        className="ml-2 w-9 h-9 rounded-md flex items-center justify-center"
                        type="button"
                        aria-label="Toggle Fullscreen"
                    >
                        {isFullScreen ? <FaCompress /> : <FaExpand />}
                    </button>

                    <button
                        type="button"
                        className="w-9 h-9 rounded-md flex items-center justify-center"
                        aria-pressed={isLocked}
                        aria-label={isLocked ? "Unlock console" : "Lock console"}
                        onMouseDown={(e) => e.preventDefault()}
                        onClick={handleToggleLock}
                        title={isLocked ? "Unlock console" : "Lock console"}
                        style={{ border: "1px solid rgba(255,255,255,0.4)", background: "rgba(0,0,0,0.12)", color: "#fff" }}
                    >
                        {isLocked ? <FaLock /> : <FaUnlock />}
                    </button>

                    <button
                        type="button"
                        className="w-9 h-9 rounded-md flex items-center justify-center"
                        onClick={handleLogout}
                        title="Log out"
                        aria-label="Log out"
                        style={{ border: "1px solid rgba(255,255,255,0.4)", background: "rgba(0,0,0,0.12)", color: "#fff" }}
                    >
                        <FaSignOutAlt />
                    </button>
                </div>
            </header>

            {/* lock hint overlay and fixed unlock button */}
            {isLocked && (
                <>
                    <div
                        className="fixed inset-0 z-[1200] flex items-center justify-center"
                        style={{ background: "rgba(0,0,0,0.5)" }}
                        role="dialog"
                        aria-modal="true"
                        onClick={(e) => e.stopPropagation()}
                    >
                        <div style={{ background: "rgba(255,255,255,0.06)", color: "#fff", padding: "12px 16px", borderRadius: 10, fontWeight: 600, fontSize: 14, boxShadow: "0 10px 30px rgba(0,0,0,0.6)" }}>
                            Console Locked - Click unlock button to access
                        </div>
                    </div>

                    <button
                        type="button"
                        className="fixed top-3 right-3 z-[1300] flex items-center justify-center rounded-[10px]"
                        onClick={handleToggleLock}
                        aria-label={isLocked ? "Unlock console" : "Lock console"}
                        title={isLocked ? "Unlock console" : "Lock console"}
                        style={{
                            width: 44,
                            height: 44,
                            border: "2px solid rgba(255,255,255,0.8)",
                            background: "rgba(0,0,0,0.4)",
                            color: "#fff",
                            boxShadow: "0 8px 24px rgba(0,0,0,0.5)",
                            fontSize: 20,
                        }}
                    >
                        {isLocked ? <FaLock /> : <FaUnlock />}
                    </button>
                </>
            )}

            {/* battery modal */}
            {batteryModalOpen && (
                <div className="fixed inset-0 flex items-center justify-center z-[1250]" style={{ background: "rgba(25,30,38,0.6)", backdropFilter: "blur(3px)" }} onClick={() => setBatteryModalOpen(false)}>
                    <div className="bg-white text-[#0b1220] rounded-lg w-[min(560px,92%)] max-h-[80vh] overflow-hidden flex flex-col shadow-[0_48px_110px_rgba(8,12,20,0.52),0_12px_36px_rgba(8,12,20,0.24)] border" style={{ borderColor: "rgba(8,12,20,0.06)" }} onClick={(e) => e.stopPropagation()}>
                        <div className="flex items-center justify-between px-4 py-3 border-b" style={{ borderColor: "rgba(15,23,42,0.06)" }}>
                            <h3 className="m-0 text-base font-semibold" style={{ fontFamily: 'Georgia, "Times New Roman", serif' }}>Battery Status</h3>
                            <button className="px-2 py-1" onClick={() => setBatteryModalOpen(false)} aria-label="Close battery modal">✕</button>
                        </div>

                        <div className="p-4 overflow-auto flex flex-col gap-3">
                            <div className="grid grid-cols-3 gap-3">
                                <div>
                                    <span className="text-xs uppercase tracking-wider" style={{ color: "#7b8591" }}>Pack Voltage</span>
                                    <strong className="text-lg block">{batteryStatus.packVoltage} V</strong>
                                </div>
                                <div>
                                    <span className="text-xs uppercase tracking-wider" style={{ color: "#7b8591" }}>Pack Current</span>
                                    <strong className="text-lg block">{batteryStatus.packCurrent} A</strong>
                                </div>
                                <div>
                                    <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
                                        <span className="text-xs uppercase tracking-wider" style={{ color: "#7b8591" }}>State of Charge</span>
                                        <span className="inline-block w-3 h-3 rounded-full bg-gray-300" aria-hidden="true" />
                                    </div>
                                    <strong className="text-lg block">{batteryStatus.stateOfCharge}%</strong>
                                </div>

                                <div>
                                    <span className="text-xs uppercase tracking-wider" style={{ color: "#7b8591" }}>Temperature</span>
                                    <strong className="text-lg block">{batteryStatus.temperature}</strong>
                                </div>
                                <div>
                                    <span className="text-xs uppercase tracking-wider" style={{ color: "#7b8591" }}>Cycles</span>
                                    <strong className="text-lg block">{batteryStatus.cycles}</strong>
                                </div>
                                <div>
                                    <span className="text-xs uppercase tracking-wider" style={{ color: "#7b8591" }}>Health</span>
                                    <strong className="text-lg block">{batteryStatus.health}</strong>
                                </div>
                            </div>

                            <div className="mt-2 overflow-auto">
                                <table className="w-full table-fixed border-collapse" style={{ tableLayout: "fixed" }}>
                                    <thead>
                                        <tr>
                                            <th className="text-left py-2 text-xs" style={{ color: "#7b8591", fontWeight: 700 }}>Cell</th>
                                            <th className="text-left py-2 text-xs pl-2" style={{ color: "#7b8591", fontWeight: 700 }}>Voltage</th>
                                        </tr>
                                    </thead>
                                    <tbody>
                                        {(batteryStatus.cells || []).map((cell) => (
                                            <tr key={cell.id} className="border-b" style={{ borderColor: "rgba(15,23,42,0.04)" }}>
                                                <td className="py-2 text-sm">{cell.id}</td>
                                                <td className="py-2 text-sm pl-2" style={{ color: "#374151" }}>{cell.voltage} V</td>
                                            </tr>
                                        ))}
                                    </tbody>
                                </table>
                            </div>
                        </div>
                    </div>
                </div>
            )}
        </>
    );
};

export default Header;
