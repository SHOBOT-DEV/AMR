import React, { useRef, useEffect } from "react";

interface JoystickData {
  x: number;
  y: number;
  force: number;
  angle: number;
  type?: string;
}

interface JoystickProps {
  width?: number;
  height?: number;
  onMove?: (data: JoystickData) => void;
  className?: string;
}

// Extend Window interface to include JoyStick
declare global {
  interface Window {
    JoyStick?: any;
  }
}

interface JoyStickInstance {
  Destroy?: () => void;
  Release?: () => void;
  SetPosition?: (x: number, y: number) => void;
}

const Joystick: React.FC<JoystickProps> = ({
  width = 120,
  height = 120,
  onMove,
  className,
}) => {
  const containerRef = useRef<HTMLDivElement | null>(null);
  const joystickRef = useRef<JoyStickInstance | null>(null);

  const keyStateRef = useRef({
    w: false,
    a: false,
    s: false,
    d: false,
    ArrowUp: false,
    ArrowLeft: false,
    ArrowDown: false,
    ArrowRight: false,
  });

  const lastSentRef = useRef<JoystickData | null>(null);
  const rafRef = useRef<number | null>(null);
  const pressedRef = useRef(false);

  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;

    if (!container.id) {
      container.id = `joystick-${Math.random().toString(36).slice(2, 9)}`;
    }

    if (!window.JoyStick) return;

    if (joystickRef.current?.Destroy) joystickRef.current.Destroy();
    while (container.firstChild) container.removeChild(container.firstChild);

    joystickRef.current = new window.JoyStick(
      container.id,
      {
        width,
        height,
        internalFillColor: "#111",
        internalStrokeColor: "#000",
        externalStrokeColor: "#000",
        internalLineWidth: 3,
        externalLineWidth: 4,
        autoReturnToCenter: true,
      },
      (stickData: JoystickData) => {
        if (isUILocked()) {
          fullyStopKeyboard();
          return;
        }
        onMove && onMove({ ...stickData, type: "joystick" });
      }
    );

    // HELPER — detect input typing
    const anyEditableFocused = (): boolean => {
      const el = document.activeElement;
      if (!el) return false;
      const tag = (el.tagName || "").toUpperCase();
      if (["INPUT", "TEXTAREA", "SELECT"].includes(tag)) return true;
      if ((el as HTMLElement).isContentEditable) return true;
      return false;
    };

    // HELPER — detect lock state
    const isUILocked = (): boolean =>
      !!document.querySelector(".main-container.is-locked");

    // HELPER — fully stop movement
    const fullyStopKeyboard = () => {
      const ks = keyStateRef.current;
      for (const k of Object.keys(ks) as Array<keyof typeof ks>) {
        ks[k] = false;
      }

      const stop: JoystickData = { x: 0, y: 0, force: 0, angle: 0, type: "stop" };
      lastSentRef.current = stop;
      onMove && onMove(stop);

      if (joystickRef.current?.Release) {
        try {
          joystickRef.current.Release();
        } catch {}
      }
      pressedRef.current = false;
    };

    const HANDLED_KEYS = new Set([
      "w",
      "a",
      "s",
      "d",
      "W",
      "A",
      "S",
      "D",
      "ArrowUp",
      "ArrowLeft",
      "ArrowDown",
      "ArrowRight",
    ]);

    const computeKeyboardMovement = () => {
      if (isUILocked()) {
        fullyStopKeyboard();
        rafRef.current = requestAnimationFrame(computeKeyboardMovement);
        return;
      }
      if (anyEditableFocused()) {
        fullyStopKeyboard();
        rafRef.current = requestAnimationFrame(computeKeyboardMovement);
        return;
      }

      const s = keyStateRef.current;

      const lettersActive = s.w || s.a || s.s || s.d;
      const right = lettersActive ? (s.d ? 1 : 0) : s.ArrowRight ? 1 : 0;
      const left = lettersActive ? (s.a ? 1 : 0) : s.ArrowLeft ? 1 : 0;
      const down = lettersActive ? (s.s ? 1 : 0) : s.ArrowDown ? 1 : 0;
      const up = lettersActive ? (s.w ? 1 : 0) : s.ArrowUp ? 1 : 0;

      const vx = right - left;
      const vy = down - up;

      const payloadX = vx;
      const payloadY = -vy;
      const force = vx !== 0 || vy !== 0 ? 1 : 0;
      const angle = force ? (Math.atan2(payloadY, payloadX) * 180) / Math.PI : 0;

      const payload: JoystickData = {
        x: payloadX,
        y: payloadY,
        force,
        angle,
        type: "keyboard",
      };

      const last = lastSentRef.current;
      if (
        !last ||
        last.x !== payload.x ||
        last.y !== payload.y ||
        last.force !== payload.force
      ) {
        lastSentRef.current = payload;
        onMove && onMove(payload);
      }

      const canvas = container.querySelector("canvas");
      if (canvas) {
        const rect = canvas.getBoundingClientRect();
        const cx = rect.left + rect.width / 2;
        const cy = rect.top + rect.height / 2;
        const maxR = (Math.min(rect.width, rect.height) / 2) * 0.6;

        const clientPx = Math.round(cx + vx * maxR);
        const clientPy = Math.round(cy + vy * maxR);

        if (joystickRef.current?.SetPosition) {
          if (force > 0) {
            joystickRef.current.SetPosition(clientPx, clientPy);
            pressedRef.current = true;
          } else if (pressedRef.current && joystickRef.current?.Release) {
            joystickRef.current.Release();
            pressedRef.current = false;
          }
        }
      }

      rafRef.current = requestAnimationFrame(computeKeyboardMovement);
    };

    // FIXED: BLOCK KEYS WHEN LOCKED
    const onKeyDown = (e: KeyboardEvent) => {
      if (!HANDLED_KEYS.has(e.key)) return;

      if (isUILocked()) {
        fullyStopKeyboard();
        return;
      }

      if (anyEditableFocused()) return;

      const ks = keyStateRef.current;

      if (e.key.toLowerCase() === "w") ks.w = true;
      if (e.key.toLowerCase() === "a") ks.a = true;
      if (e.key.toLowerCase() === "s") ks.s = true;
      if (e.key.toLowerCase() === "d") ks.d = true;

      if (e.key === "ArrowUp") ks.ArrowUp = true;
      if (e.key === "ArrowLeft") ks.ArrowLeft = true;
      if (e.key === "ArrowDown") ks.ArrowDown = true;
      if (e.key === "ArrowRight") ks.ArrowRight = true;

      e.preventDefault();
    };

    const onKeyUp = (e: KeyboardEvent) => {
      if (!HANDLED_KEYS.has(e.key)) return;

      if (isUILocked()) {
        fullyStopKeyboard();
        return;
      }

      const ks = keyStateRef.current;

      if (e.key.toLowerCase() === "w") ks.w = false;
      if (e.key.toLowerCase() === "a") ks.a = false;
      if (e.key.toLowerCase() === "s") ks.s = false;
      if (e.key.toLowerCase() === "d") ks.d = false;

      if (e.key === "ArrowUp") ks.ArrowUp = false;
      if (e.key === "ArrowLeft") ks.ArrowLeft = false;
      if (e.key === "ArrowDown") ks.ArrowDown = false;
      if (e.key === "ArrowRight") ks.ArrowRight = false;

      e.preventDefault();
    };

    window.addEventListener("keydown", onKeyDown as any, { passive: false });
    window.addEventListener("keyup", onKeyUp as any, { passive: false });

    rafRef.current = requestAnimationFrame(computeKeyboardMovement);

    return () => {
      window.removeEventListener("keydown", onKeyDown as any);
      window.removeEventListener("keyup", onKeyUp as any);
      if (rafRef.current) cancelAnimationFrame(rafRef.current);
      if (joystickRef.current?.Destroy) joystickRef.current.Destroy();
      joystickRef.current = null;
    };
  }, [width, height, onMove]);

  return (
    <div
      id="joystick-container"
      ref={containerRef}
      className={`block touch-none ${className || ""}`}
      tabIndex={0}
      style={{
        width: `${width}px`,
        height: `${height}px`,
      }}
    />
  );
};

export default Joystick;
