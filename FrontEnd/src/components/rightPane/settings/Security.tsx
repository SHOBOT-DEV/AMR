import React, { useCallback, useState } from "react";

/* ===================== TYPES ===================== */
interface SecurityPreferences {
  twoFactor: boolean;
  autoLock: boolean;
  anomalyAlerts: boolean;
}

interface SecurityEvent {
  id: string;
  ts: string;
  actor: string;
  action: string;
  context: string;
}

interface SecurityProps {
  rightPage: string;
  requestV1: (url: string, options?: RequestInit) => Promise<any>;
  toast: {
    error: (msg: string) => void;
  };
}

/* ===================== COMPONENT ===================== */
const Security: React.FC<SecurityProps> = ({
  rightPage,
  requestV1,
  toast,
}) => {
  /* ===================== STATE ===================== */
  const [securityPreferences, setSecurityPreferences] =
    useState<SecurityPreferences>({
      twoFactor: true,
      autoLock: true,
      anomalyAlerts: true,
    });

  const [securityEvents, setSecurityEvents] = useState<SecurityEvent[]>([
    {
      id: "sec1",
      ts: "09:44",
      actor: "ops-admin",
      action: "API token created",
      context: "Main console",
    },
    {
      id: "sec2",
      ts: "08:12",
      actor: "robot-01",
      action: "Cert renewed",
      context: "Device",
    },
  ]);

  /* ===================== API PERSIST ===================== */
  const persistSecurityPref = useCallback(
    (
      key: keyof SecurityPreferences,
      value: boolean,
      previousValue: boolean
    ) => {
      requestV1("/settings/security", {
        method: "PATCH",
        body: JSON.stringify({ [key]: value }),
      }).catch((error) => {
        console.error("Security pref update failed", error);
        toast.error(
          error?.message || "Failed to update security preference"
        );
        setSecurityPreferences((prev) => ({
          ...prev,
          [key]: previousValue,
        }));
      });
    },
    [requestV1, toast]
  );

  /* ===================== TOGGLE ===================== */
  const toggleSecurityPref = (key: keyof SecurityPreferences) => {
    setSecurityPreferences((prev) => {
      const nextValue = !prev[key];
      persistSecurityPref(key, nextValue, prev[key]);
      return {
        ...prev,
        [key]: nextValue,
      };
    });
  };

  /* ===================== PAGE GUARD ===================== */
  if (rightPage !== "security") return null;

  /* ===================== UI ===================== */
  return (
    <div className="flex flex-col gap-6 p-6">
      {/* Toggles */}
      <div className="grid gap-4 md:grid-cols-3">
        {Object.entries(securityPreferences).map(([key, value]) => (
          <label
            key={key}
            className="flex items-center gap-4 p-4 border rounded-xl cursor-pointer hover:bg-gray-50"
          >
            <input
              type="checkbox"
              checked={value}
              onChange={() =>
                toggleSecurityPref(key as keyof SecurityPreferences)
              }
              className="h-5 w-5 accent-blue-600"
            />
            <div>
              <p className="font-semibold text-gray-900">
                {key.replace(/([A-Z])/g, " $1")}
              </p>
              <p className="text-sm text-gray-500">
                {value ? "Enabled" : "Disabled"}
              </p>
            </div>
          </label>
        ))}
      </div>

      {/* Security Events Table */}
      <div className="overflow-x-auto rounded-xl border">
        <table className="w-full border-collapse">
          <thead className="bg-gray-100">
            <tr>
              <th className="px-4 py-3 text-left text-sm font-semibold text-gray-700">
                Time
              </th>
              <th className="px-4 py-3 text-left text-sm font-semibold text-gray-700">
                Actor
              </th>
              <th className="px-4 py-3 text-left text-sm font-semibold text-gray-700">
                Action
              </th>
              <th className="px-4 py-3 text-left text-sm font-semibold text-gray-700">
                Context
              </th>
            </tr>
          </thead>
          <tbody>
            {securityEvents.map((evt) => (
              <tr
                key={evt.id}
                className="border-t hover:bg-gray-50"
              >
                <td className="px-4 py-3 text-sm text-gray-600">
                  {evt.ts}
                </td>
                <td className="px-4 py-3 text-sm font-medium text-gray-900">
                  {evt.actor}
                </td>
                <td className="px-4 py-3 text-sm text-gray-700">
                  {evt.action}
                </td>
                <td className="px-4 py-3 text-sm text-gray-500">
                  {evt.context}
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </div>
  );
};

export default Security;
