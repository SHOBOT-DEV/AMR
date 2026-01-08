import React, { createContext, useContext, useState } from "react";

type OverviewStats = { totalKm: number; missionsCompleted: number; avgSpeed: number; operatingHours: number };
type MissionTrendEntry = { label: string; completed: number; incidents: number };
type MonthlyMovementEntry = { month: string; km: number };
type BatteryPoint = { time: string; voltage: number; power: number };
type BatteryStatus = { packVoltage: number; packCurrent: number; stateOfCharge: number; temperature: string; cycles: number; health: string; cells?: Array<{ id: string; voltage: number }> };
type StatsData = { overview: OverviewStats & { deltaKm?: number; missionSuccessRate?: number }; batterySeries: BatteryPoint[]; batteryStatus: BatteryStatus; missionTrend: MissionTrendEntry[]; monthlyMovement: MonthlyMovementEntry[]; turns: { left: number; right: number } };

const FALLBACK_STATS: StatsData = {
    overview: { totalKm: 182.4, missionsCompleted: 47, avgSpeed: 1.8, operatingHours: 326, deltaKm: 4.3, missionSuccessRate: 98 },
    batterySeries: [{ time: "08:00", voltage: 48.2, power: 182 }, { time: "09:00", voltage: 47.8, power: 176 }, { time: "10:00", voltage: 47.4, power: 171 }, { time: "11:00", voltage: 46.9, power: 168 }, { time: "12:00", voltage: 47.1, power: 170 }, { time: "13:00", voltage: 46.7, power: 166 }, { time: "14:00", voltage: 46.3, power: 164 }],
    batteryStatus: { packVoltage: 46.9, packCurrent: 38.2, stateOfCharge: 78, temperature: "32°C", cycles: 412, health: "Good", cells: [{ id: "Cell A", voltage: 3.9 }, { id: "Cell B", voltage: 3.89 }, { id: "Cell C", voltage: 3.88 }, { id: "Cell D", voltage: 3.87 }] },
    missionTrend: [{ label: "Mon", completed: 5, incidents: 0 }, { label: "Tue", completed: 7, incidents: 1 }, { label: "Wed", completed: 6, incidents: 0 }, { label: "Thu", completed: 8, incidents: 1 }, { label: "Fri", completed: 9, incidents: 0 }],
    monthlyMovement: [{ month: "Jan", km: 118 }, { month: "Feb", km: 142 }, { month: "Mar", km: 131 }, { month: "Apr", km: 155 }, { month: "May", km: 162 }, { month: "Jun", km: 174 }],
    turns: { left: 132, right: 148 },
};

type StatsContextShape = { statsData: StatsData; setStatsData: React.Dispatch<React.SetStateAction<StatsData>> };

const StatsContext = createContext<StatsContextShape | undefined>(undefined);

export const StatsProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
    const [statsData, setStatsData] = useState<StatsData>(FALLBACK_STATS);
    return <StatsContext.Provider value={{ statsData, setStatsData }}>{children}</StatsContext.Provider>;
};

export const useStats = (): StatsContextShape => {
    const ctx = useContext(StatsContext);
    if (!ctx) throw new Error("useStats must be used within StatsProvider");
    return ctx;
};