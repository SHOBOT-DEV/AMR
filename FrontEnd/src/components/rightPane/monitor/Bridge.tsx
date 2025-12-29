import React from "react";

interface BridgeStatus {
  connected: boolean;
  endpoint: string;
  latency?: number;
  lastSync?: string;
  error?: string;
}

interface BridgeProps {
  bridgeStatus?: BridgeStatus;
}

const Bridge: React.FC<BridgeProps> = ({
  bridgeStatus = {
    connected: true,
    endpoint: "ws://amr-bridge.local:8080",
    latency: 12,
    lastSync: "2025-12-29 14:32:15",
  },
}) => {
  return (
    <div className="flex flex-col gap-4">
      {/* Connection Status Card */}
      <div
        className={`border rounded-lg p-4 ${
          bridgeStatus.connected
            ? "bg-green-50 border-green-200"
            : "bg-red-50 border-red-200"
        }`}
      >
        <div className="flex items-center justify-between mb-3">
          <strong className="text-sm font-semibold text-slate-900">
            Connection Status
          </strong>
          <div
            className={`w-3 h-3 rounded-full ${
              bridgeStatus.connected ? "bg-green-500 animate-pulse" : "bg-red-500"
            }`}
          />
        </div>
        <p
          className={`text-sm font-medium ${
            bridgeStatus.connected ? "text-green-700" : "text-red-700"
          }`}
        >
          {bridgeStatus.connected ? "Connected" : "Disconnected"}
        </p>
      </div>

      {/* Bridge Details */}
      <div className="bg-white border border-slate-200 rounded-lg p-4 space-y-3">
        <div>
          <label className="text-xs font-semibold text-slate-500 uppercase tracking-wide block mb-1">
            Endpoint
          </label>
          <p className="text-sm font-mono text-slate-700 break-all">
            {bridgeStatus.endpoint}
          </p>
        </div>

        {bridgeStatus.latency !== undefined && (
          <div>
            <label className="text-xs font-semibold text-slate-500 uppercase tracking-wide block mb-1">
              Latency
            </label>
            <p className="text-sm text-slate-700">
              {bridgeStatus.latency}
              <span className="text-slate-500 text-xs ml-1">ms</span>
            </p>
          </div>
        )}

        {bridgeStatus.lastSync && (
          <div>
            <label className="text-xs font-semibold text-slate-500 uppercase tracking-wide block mb-1">
              Last Sync
            </label>
            <p className="text-sm text-slate-700">{bridgeStatus.lastSync}</p>
          </div>
        )}

        {bridgeStatus.error && (
          <div className="bg-red-50 border border-red-200 rounded p-2 mt-3">
            <p className="text-xs text-red-700">Error: {bridgeStatus.error}</p>
          </div>
        )}
      </div>

      {/* Actions */}
      <div className="flex gap-2">
        <button className="flex-1 px-4 py-2 bg-slate-100 hover:bg-slate-200 text-slate-900 font-medium rounded-lg text-sm transition-colors">
          Reconnect
        </button>
        <button className="flex-1 px-4 py-2 bg-blue-600 hover:bg-blue-700 text-white font-medium rounded-lg text-sm transition-colors">
          Test Connection
        </button>
      </div>
    </div>
  );
};

export default Bridge;
