import React, { useRef, useEffect } from "react";

const Joystick = ({ width = 120, height = 120, onMove, className }) => {
  const containerRef = useRef(null);
  const joystickRef = useRef(null);

  // 1. Keep track of the latest onMove callback without triggering effects
  const onMoveRef = useRef(onMove);
  useEffect(() => {
    onMoveRef.current = onMove;
  }, [onMove]);

  const keyStateRef = useRef({
    w: false, a: false, s: false, d: false,
    ArrowUp: false, ArrowLeft: false, ArrowDown: false, ArrowRight: false,
  });

  const lastSentRef = useRef(null);
  const rafRef = useRef(null);
  const pressedRef = useRef(false);

  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;

    if (!container.id) {
      container.id = `joystick-${Math.random().toString(36).slice(2, 9)}`;
    }

    if (!window.JoyStick) return;

    // Cleanup previous instance if valid
    if (joystickRef.current?.Destroy) joystickRef.current.Destroy();
    while (container.firstChild) container.removeChild(container.firstChild);

    // Initialize Joystick
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
      (stickData) => {
        // Helper check inside callback
        const isLocked = !!document.querySelector(".main-container.is-locked");
        
        if (isLocked) {
          // Logic to stop if locked (simplified for brevity)
          return; 
        }
        // Call the Ref, not the prop directly
        if (onMoveRef.current) {
          onMoveRef.current({ ...stickData, type: "joystick" });
        }
      }
    );

    // --- Helpers ---
    const anyEditableFocused = () => {
      const el = document.activeElement;
      if (!el) return false;
      const tag = (el.tagName || "").toUpperCase();
      return ["INPUT", "TEXTAREA", "SELECT"].includes(tag) || el.isContentEditable;
    };

    const isUILocked = () => !!document.querySelector(".main-container.is-locked");

    const fullyStopKeyboard = () => {
      const ks = keyStateRef.current;
      Object.keys(ks).forEach(k => ks[k] = false);

      const stop = { x: 0, y: 0, force: 0, angle: 0, type: "stop" };
      
      // Only send stop if we weren't already stopped
      if (lastSentRef.current?.force !== 0) {
        lastSentRef.current = stop;
        if (onMoveRef.current) onMoveRef.current(stop);
      }

      if (pressedRef.current && joystickRef.current?.Release) {
        try { joystickRef.current.Release(); } catch (e) {}
      }
      pressedRef.current = false;
    };

    const HANDLED_KEYS = new Set(["w", "a", "s", "d", "ArrowUp", "ArrowLeft", "ArrowDown", "ArrowRight"]);

    // --- Game Loop ---
    const computeKeyboardMovement = () => {
      if (isUILocked() || anyEditableFocused()) {
        if (pressedRef.current) fullyStopKeyboard();
        rafRef.current = requestAnimationFrame(computeKeyboardMovement);
        return;
      }

      const s = keyStateRef.current;
      
      // Check if any key is actually pressed to avoid unnecessary calculation
      const anyKeyPressed = Object.values(s).some(v => v);
      
      // Optimization: If no keys pressed and already stopped, skip calculation
      if (!anyKeyPressed && !pressedRef.current) {
         rafRef.current = requestAnimationFrame(computeKeyboardMovement);
         return;
      }

      const lettersActive = s.w || s.a || s.s || s.d;
      const right = lettersActive ? (s.d ? 1 : 0) : (s.ArrowRight ? 1 : 0);
      const left = lettersActive ? (s.a ? 1 : 0) : (s.ArrowLeft ? 1 : 0);
      const down = lettersActive ? (s.s ? 1 : 0) : (s.ArrowDown ? 1 : 0);
      const up = lettersActive ? (s.w ? 1 : 0) : (s.ArrowUp ? 1 : 0);

      const vx = right - left;
      const vy = down - up;

      const payloadX = vx;
      const payloadY = -vy;
      const force = vx !== 0 || vy !== 0 ? 1 : 0;
      const angle = force ? (Math.atan2(payloadY, payloadX) * 180) / Math.PI : 0;

      const payload = { x: payloadX, y: payloadY, force, angle, type: "keyboard" };

      // Deduping logic
      const last = lastSentRef.current;
      if (
        !last ||
        last.x !== payload.x ||
        last.y !== payload.y ||
        last.force !== payload.force
      ) {
        lastSentRef.current = payload;
        if (onMoveRef.current) onMoveRef.current(payload);
      }

      // Visual Joystick Update
      if (force > 0) {
        const canvas = container.querySelector("canvas");
        if (canvas && joystickRef.current?.SetPosition) {
          const rect = canvas.getBoundingClientRect();
          const cx = rect.left + rect.width / 2;
          const cy = rect.top + rect.height / 2;
          const maxR = (Math.min(rect.width, rect.height) / 2) * 0.6;
          
          const clientPx = Math.round(cx + vx * maxR);
          const clientPy = Math.round(cy + vy * maxR);
          
          joystickRef.current.SetPosition(clientPx, clientPy);
          pressedRef.current = true;
        }
      } else if (pressedRef.current) {
        if (joystickRef.current?.Release) joystickRef.current.Release();
        pressedRef.current = false;
      }

      rafRef.current = requestAnimationFrame(computeKeyboardMovement);
    };

    // --- Event Listeners ---
    const onKeyDown = (e) => {
      if (!HANDLED_KEYS.has(e.key) && !HANDLED_KEYS.has(e.key.replace("Arrow", ""))) return;
      if (isUILocked() || anyEditableFocused()) return;

      const k = e.key.toLowerCase();
      const ks = keyStateRef.current;
      
      if (k === "w") ks.w = true;
      if (k === "a") ks.a = true;
      if (k === "s") ks.s = true;
      if (k === "d") ks.d = true;
      if (e.key === "ArrowUp") ks.ArrowUp = true;
      if (e.key === "ArrowLeft") ks.ArrowLeft = true;
      if (e.key === "ArrowDown") ks.ArrowDown = true;
      if (e.key === "ArrowRight") ks.ArrowRight = true;
    };

    const onKeyUp = (e) => {
      if (!HANDLED_KEYS.has(e.key) && !HANDLED_KEYS.has(e.key.replace("Arrow", ""))) return;
      
      const k = e.key.toLowerCase();
      const ks = keyStateRef.current;

      if (k === "w") ks.w = false;
      if (k === "a") ks.a = false;
      if (k === "s") ks.s = false;
      if (k === "d") ks.d = false;
      if (e.key === "ArrowUp") ks.ArrowUp = false;
      if (e.key === "ArrowLeft") ks.ArrowLeft = false;
      if (e.key === "ArrowDown") ks.ArrowDown = false;
      if (e.key === "ArrowRight") ks.ArrowRight = false;
    };

    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("keyup", onKeyUp);
    rafRef.current = requestAnimationFrame(computeKeyboardMovement);

    return () => {
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("keyup", onKeyUp);
      if (rafRef.current) cancelAnimationFrame(rafRef.current);
      if (joystickRef.current?.Destroy) joystickRef.current.Destroy();
    };
    
    // DEPENDENCY ARRAY CHANGED: Removed 'onMove'
  }, [width, height]); 

  return (
    <div
      id="joystick-container"
      ref={containerRef}
      className={className}
      tabIndex={0}
      style={{
        width: `${width}px`,
        height: `${height}px`,
        touchAction: "none",
        display: "block",
      }}
    />
  );
};

// 2. Prevent re-renders when Parent state changes
export default React.memo(Joystick);