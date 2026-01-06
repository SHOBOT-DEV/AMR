import React, { createContext, useContext, useState, ReactNode } from "react";

// --- Types ---
export type OverviewStats = {
  totalKm: number;
  missionsCompleted: number;
  avgSpeed: number;
  operatingHours: number;
};

export type BatteryPoint = {
  time: string;
  voltage: number;
  power: number;
};

export type BatteryStatus = {
  packVoltage: number;
  packCurrent: number;
  stateOfCharge: number;
  temperature: string;
  cycles: number;
  health: string;
  cells: { id: string; voltage: number }[];
};

export type MissionTrendEntry = {
  label: string;
  completed: number;
  incidents: number;
};

export type MonthlyMovementEntry = {
  month: string;
  km: number;
};

export type StatsData = {
  overview: OverviewStats & { deltaKm?: number; missionSuccessRate?: number };
  batterySeries: BatteryPoint[];
  batteryStatus: BatteryStatus;
  missionTrend: MissionTrendEntry[];
  monthlyMovement: MonthlyMovementEntry[];
  turns: { left: number; right: number };
};

// --- Fallback Stats ---
const FALLBACK_STATS: StatsData = {
  overview: {
    totalKm: 182.4,
    missionsCompleted: 47,
    avgSpeed: 1.8,
    operatingHours: 326,
    deltaKm: 4.3,
    missionSuccessRate: 98,
  },
  batterySeries: [
    { time: "08:00", voltage: 48.2, power: 182 },
    { time: "09:00", voltage: 47.8, power: 176 },
    { time: "10:00", voltage: 47.4, power: 171 },
    { time: "11:00", voltage: 46.9, power: 168 },
    { time: "12:00", voltage: 47.1, power: 170 },
    { time: "13:00", voltage: 46.7, power: 166 },
    { time: "14:00", voltage: 46.3, power: 164 },
  ],
  batteryStatus: {
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
  missionTrend: [
    { label: "Mon", completed: 5, incidents: 0 },
    { label: "Tue", completed: 7, incidents: 1 },
    { label: "Wed", completed: 6, incidents: 0 },
    { label: "Thu", completed: 8, incidents: 1 },
    { label: "Fri", completed: 9, incidents: 0 },
  ],
  monthlyMovement: [
    { month: "Jan", km: 118 },
    { month: "Feb", km: 142 },
    { month: "Mar", km: 131 },
    { month: "Apr", km: 155 },
    { month: "May", km: 162 },
    { month: "Jun", km: 174 },
  ],
  turns: { left: 132, right: 148 },
};

// --- Context ---
interface StatsContextType {
  statsData: StatsData;
  setStatsData: React.Dispatch<React.SetStateAction<StatsData>>;
  statsLoading: boolean;
  setStatsLoading: React.Dispatch<React.SetStateAction<boolean>>;
  statsError: string;
  setStatsError: React.Dispatch<React.SetStateAction<string>>;
}

const StatsContext = createContext<StatsContextType | undefined>(undefined);

// --- Provider ---
export const StatsProvider = ({ children }: { children: ReactNode }) => {
  const [statsData, setStatsData] = useState<StatsData>(FALLBACK_STATS);
  const [statsLoading, setStatsLoading] = useState(false);
  const [statsError, setStatsError] = useState("");

  return (
    <StatsContext.Provider
      value={{
        statsData,
        setStatsData,
        statsLoading,
        setStatsLoading,
        statsError,
        setStatsError,
      }}
    >
      {children}
    </StatsContext.Provider>
  );
};

// --- Hook ---
export const useStats = () => {
  const context = useContext(StatsContext);
  if (!context) throw new Error("useStats must be used within StatsProvider");
  return context;
};
