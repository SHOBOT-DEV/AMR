import React, { createContext, useContext, useState } from "react";

type BridgeStatus = { connected: boolean; endpoint: string; error?: string };

const SAMPLE_ANALYTICS_SUMMARY = [{ label: "Incidents", value: 2, trend: "+1 vs last week" }, { label: "Stops Issued", value: 14, trend: "-3 vs last week" }, { label: "Battery Swaps", value: 5, trend: "Stable" }];

const SAMPLE_ANALYTICS_SERIES = [12, 18, 22, 16, 24, 26, 20];

const SAMPLE_ANALYTICS_ALERTS = [{ id: "alert1", title: "Obstacle spikes", detail: "Lidar reported 5 high-density events on Dock Tunnel." }, { id: "alert2", title: "Slow return", detail: "Mission Delivery Route 3 exceeded SLA by 4 min." }];

const SAMPLE_DIAGNOSTICS = [{ id: "battery", title: "Battery Health", value: "93%", status: "Nominal", detail: "Cells balanced" }, { id: "motors", title: "Drive Motors", value: "Temp 48°C", status: "Monitoring", detail: "Torque variance +3%" }];

const SAMPLE_LOG_EVENTS = [{ id: "log1", ts: "10:42:01", system: "Navigation", message: "Replanned path around blocked aisle", level: "info" }, { id: "log2", ts: "10:15:22", system: "Safety", message: "Emergency stop acknowledged", level: "warn" }];

const SAMPLE_MISSION_HISTORY = [{ id: "mh1", mission: "Inspect Zone A", window: "08:00–08:18", outcome: "Completed", notes: "No issues" }, { id: "mh2", mission: "Delivery Route 3", window: "08:30–09:10", outcome: "Delayed", notes: "Obstacle at Dock Tunnel" }];

const SAMPLE_BAG_FILES = [{ id: "bag1", name: "mission-0915.bag", duration: "15m", size: "1.4 GB", status: "Uploaded" }, { id: "bag2", name: "mission-1030.bag", duration: "26m", size: "2.7 GB", status: "Processing" }];

type MonitorContextShape = {
    analyticsSummary: any[];
    analyticsSeries: number[];
    analyticsAlerts: any[];
    diagnosticsPanels: any[];
    logEvents: any[];
    missionHistory: any[];
    bagFiles: any[];
    bridgeStatus: BridgeStatus;
    setBridgeStatus: React.Dispatch<React.SetStateAction<BridgeStatus>>;
};

const MonitorContext = createContext<MonitorContextShape | undefined>(undefined);

export const MonitorProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
    const [analyticsSummary] = useState<any[]>(SAMPLE_ANALYTICS_SUMMARY);
    const [analyticsSeries] = useState<number[]>(SAMPLE_ANALYTICS_SERIES);
    const [analyticsAlerts] = useState<any[]>(SAMPLE_ANALYTICS_ALERTS);
    const [diagnosticsPanels] = useState<any[]>(SAMPLE_DIAGNOSTICS);
    const [logEvents] = useState<any[]>(SAMPLE_LOG_EVENTS);
    const [missionHistory] = useState<any[]>(SAMPLE_MISSION_HISTORY);
    const [bagFiles] = useState<any[]>(SAMPLE_BAG_FILES);
    const [bridgeStatus, setBridgeStatus] = useState<BridgeStatus>({ connected: false, endpoint: "http://localhost:8000", error: "" });

    return (
        <MonitorContext.Provider value={{ analyticsSummary, analyticsSeries, analyticsAlerts, diagnosticsPanels, logEvents, missionHistory, bagFiles, bridgeStatus, setBridgeStatus }}>
            {children}
        </MonitorContext.Provider>
    );
};

export const useMonitor = (): MonitorContextShape => {
    const ctx = useContext(MonitorContext);
    if (!ctx) throw new Error("useMonitor must be used within MonitorProvider");
    return ctx;
};
