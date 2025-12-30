import React, { useCallback, useState } from "react";

type IntegrationStatus = "Connected" | "Disconnected";

interface IntegrationItem {
  id: string;
  name: string;
  status: IntegrationStatus;
  description: string;
}

interface IntegrationsProps {
  rightPage: string;
  requestV1: (url: string, options?: RequestInit) => Promise<any>;
  toast: {
    error: (msg: string) => void;
  };
}

const Integrations: React.FC<IntegrationsProps> = ({
  rightPage,
  requestV1,
  toast,
}) => {
 
  const [integrationItems, setIntegrationItems] = useState<IntegrationItem[]>([
    {
      id: "rest",
      name: "REST API",
      status: "Connected",
      description: "Push missions from MES",
    },
    {
      id: "slack",
      name: "Slack Bot",
      status: "Disconnected",
      description: "Alerts to #robot-ops",
    },
    {
      id: "grafana",
      name: "Grafana",
      status: "Connected",
      description: "Telemetry dashboards",
    },
  ]);
  const persistIntegrationStatus = useCallback(
    (
      id: string,
      status: IntegrationStatus,
      previousStatus: IntegrationStatus
    ) => {
      requestV1(`/settings/integrations/${id}`, {
        method: "PATCH",
        body: JSON.stringify({ status }),
      }).catch((error) => {
        console.error("Integration status update failed", error);
        toast.error(error?.message || "Failed to update integration");
        setIntegrationItems((items) =>
          items.map((item) =>
            item.id === id ? { ...item, status: previousStatus } : item
          )
        );
      });
    },
    [requestV1, toast]
  );

  const toggleIntegrationStatus = (id: string) => {
    setIntegrationItems((items) =>
      items.map((item) => {
        if (item.id !== id) return item;

        const nextStatus: IntegrationStatus =
          item.status === "Connected" ? "Disconnected" : "Connected";

        persistIntegrationStatus(id, nextStatus, item.status);

        return {
          ...item,
          status: nextStatus,
        };
      })
    );
  };

if (rightPage !== "Integrations");
  return (
    <div className="flex flex-col gap-4 p-6">
      {integrationItems.map((integration) => (
        <div
          key={integration.id}
          className="flex items-center justify-between rounded-xl border p-4 hover:bg-gray-50"
        >
          {/* Info */}
          <div>
            <p className="font-semibold text-gray-900">
              {integration.name}
            </p>
            <p className="text-sm text-gray-500">
              {integration.description}
            </p>
          </div>

          {/* Status + Action */}
          <div className="flex items-center gap-4">
            <span
              className={`px-3 py-1 rounded-full text-xs font-semibold ${
                integration.status === "Connected"
                  ? "bg-green-100 text-green-700"
                  : "bg-gray-200 text-gray-600"
              }`}
            >
              {integration.status}
            </span>

            <button
              onClick={() => toggleIntegrationStatus(integration.id)}
              className="text-sm font-medium text-blue-600 hover:underline"
            >
              {integration.status === "Connected"
                ? "Disconnect"
                : "Connect"}
            </button>
          </div>
        </div>
      ))}
    </div>
  );
};

export default Integrations;
