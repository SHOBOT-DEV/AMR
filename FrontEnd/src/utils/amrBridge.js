const DEFAULT_BASE = process.env.REACT_APP_AMR_BRIDGE_BASE || "http://localhost:8000";

const normalizeWsUrl = (baseUrl = DEFAULT_BASE) => {
  try {
    const url = new URL(baseUrl);
    url.protocol = url.protocol === "https:" ? "wss:" : "ws:";
    url.pathname = url.pathname.endsWith("/") ? `${url.pathname}ws` : `${url.pathname}/ws`;
    return url.toString();
  } catch (err) {
    console.error("Invalid AMR bridge URL:", baseUrl, err);
    return null;
  }
};

export const AMR_BRIDGE_BASE = DEFAULT_BASE;

export const fetchBridgeStatus = async (baseUrl = DEFAULT_BASE) => {
  const res = await fetch(`${baseUrl}/api/status`);
  if (!res.ok) {
    const text = await res.text().catch(() => "");
    throw new Error(text || `Status request failed (${res.status})`);
  }
  return res.json();
};

export const sendCmdVel = async (
  { linear_x = 0, linear_y = 0, angular_z = 0 } = {},
  baseUrl = DEFAULT_BASE
) => {
  const res = await fetch(`${baseUrl}/api/cmd_vel?linear_x=${linear_x}&linear_y=${linear_y}&angular_z=${angular_z}`, {
    method: "POST",
  });
  if (!res.ok) {
    const text = await res.text().catch(() => "");
    throw new Error(text || `cmd_vel request failed (${res.status})`);
  }
  return res.json();
};

export const connectBridgeSocket = ({
  baseUrl = DEFAULT_BASE,
  onOpen,
  onClose,
  onStatus,
  onError,
} = {}) => {
  const wsUrl = normalizeWsUrl(baseUrl);
  if (!wsUrl || typeof WebSocket === "undefined") {
    onError?.(new Error("WebSocket not available in this environment"));
    return { socket: null, close: () => {}, send: () => false };
  }

  let socket;
  try {
    socket = new WebSocket(wsUrl);
  } catch (err) {
    onError?.(err);
    return { socket: null, close: () => {}, send: () => false };
  }

  socket.onopen = () => onOpen?.();
  socket.onmessage = (event) => {
    try {
      const payload = JSON.parse(event.data);
      if (payload?.type === "status") {
        onStatus?.(payload.data || {});
      }
    } catch (err) {
      console.warn("Bridge socket parse error:", err);
    }
  };
  socket.onclose = () => onClose?.();
  socket.onerror = (event) => {
    const err = event?.message ? new Error(event.message) : new Error("WebSocket error");
    onError?.(err);
  };

  const send = (message) => {
    if (!socket || socket.readyState !== WebSocket.OPEN) return false;
    try {
      socket.send(JSON.stringify(message));
      return true;
    } catch (err) {
      onError?.(err);
      return false;
    }
  };

  return {
    socket,
    close: () => {
      try {
        socket?.close();
      } catch {}
    },
    send,
  };
};
