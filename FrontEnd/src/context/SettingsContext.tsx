import React, { createContext, useContext, useState } from "react";

const SAMPLE_SECURITY_EVENTS = [{ id: "sec1", ts: "09:44", actor: "ops-admin", action: "API token created", context: "Main console" }, { id: "sec2", ts: "08:12", actor: "robot-01", action: "Cert renewed", context: "Device" }];

const SAMPLE_INTEGRATIONS = [{ id: "rest", name: "REST API", status: "Connected", description: "Push missions from MES" }, { id: "slack", name: "Slack Bot", status: "Disconnected", description: "Alerts to #robot-ops" }, { id: "grafana", name: "Grafana", status: "Connected", description: "Telemetry dashboards" }];

type SettingsContextShape = {
    securityEvents: any[];
    integrationItems: any[];
    setIntegrationItems: React.Dispatch<React.SetStateAction<any[]>>;
};

const SettingsContext = createContext<SettingsContextShape | undefined>(undefined);

export const SettingsProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
    const [securityEvents] = useState<any[]>(SAMPLE_SECURITY_EVENTS);
    const [integrationItems, setIntegrationItems] = useState<any[]>(SAMPLE_INTEGRATIONS);
    return <SettingsContext.Provider value={{ securityEvents, integrationItems, setIntegrationItems }}>{children}</SettingsContext.Provider>;
};

export const useSettings = (): SettingsContextShape => {
    const ctx = useContext(SettingsContext);
    if (!ctx) throw new Error("useSettings must be used within SettingsProvider");
    return ctx;
};
