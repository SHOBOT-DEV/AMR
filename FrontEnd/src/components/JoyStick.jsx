import { useRef, useEffect } from "react";

const Joystick = ({ width = 120, height = 120, onMove, className }) => {
  const containerRef = useRef(null);
  const joystickRef = useRef(null);

  // Keyboard state
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

  const lastSentRef = useRef(null);
  const rafRef = useRef(null);
  const pressedRef = useRef(false); // for synthetic mouse press

  useEffect(() => {
    const container = containerRef.current;
    if (!container || !window.JoyStick) return;

    // Cleanup old joystick
    if (joystickRef.current?.Destroy) joystickRef.current.Destroy();
    while (container.firstChild) container.removeChild(container.firstChild);

    // Create new joystick
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
        onMove && onMove({ ...stickData, type: "joystick" });
      }
    );

    // Helper: create MouseEvent
    const makeMouseEvent = (type, x, y) => {
      try {
        return new MouseEvent(type, {
          bubbles: true,
          cancelable: true,
          view: window,
          clientX: x,
          clientY: y,
        });
      } catch {
        const ev = document.createEvent("MouseEvents");
        ev.initMouseEvent(
          type,
          true,
          true,
          window,
          0,
          x,
          y,
          x,
          y,
          false,
          false,
          false,
          false,
          0,
          null
        );
        return ev;
      }
    };

    const dispatchMouse = (target, type, x, y) => {
      try {
        target.dispatchEvent(makeMouseEvent(type, x, y));
      } catch {}
    };

    // Compute keyboard vector and synthesize native events to the joystick canvas
    const computeKeyboardMovement = () => {
      const s = keyStateRef.current;

      // Prefer letter keys (W/A/S/D). If none pressed, fall back to arrows.
      const lettersActive = s.w || s.a || s.s || s.d;
      const right = lettersActive ? (s.d ? 1 : 0) : (s.ArrowRight ? 1 : 0);
      const left  = lettersActive ? (s.a ? 1 : 0) : (s.ArrowLeft  ? 1 : 0);
      const down  = lettersActive ? (s.s ? 1 : 0) : (s.ArrowDown  ? 1 : 0);
      const up    = lettersActive ? (s.w ? 1 : 0) : (s.ArrowUp    ? 1 : 0);

      // Discrete mapping: vx = D - A (right positive), vy = S - W (down positive)
      const vx = right - left; // -1, 0, 1
      const vy = down - up;    // -1,0,1  (positive = down)

      // onMove expects x right-positive, y up-positive (joy.js convention: y is inverted)
      const payloadX = vx;
      const payloadY = -vy; // invert so up is positive

      const force = vx !== 0 || vy !== 0 ? 1 : 0;
      const angle = force ? (Math.atan2(payloadY, payloadX) * 180) / Math.PI : 0;

      // forward onMove when changed
      const payload = { x: payloadX, y: payloadY, force, angle, type: "keyboard" };
      const last = lastSentRef.current;
      if (!last || last.x !== payload.x || last.y !== payload.y || last.force !== payload.force) {
        lastSentRef.current = payload;
        onMove && onMove(payload);
      }

      // Find canvas created by joy.js and update visuals.
      const canvas = container.querySelector("canvas");
      if (canvas) {
        const rect = canvas.getBoundingClientRect();
        const cx = rect.left + rect.width / 2;
        const cy = rect.top + rect.height / 2;
        const maxR = (Math.min(rect.width, rect.height) / 2) * 0.6;

        // compute viewport (client) coordinates:
        // vx positive => move right; vy positive => move down (client Y increases downward)
        const clientPx = Math.round(cx + vx * maxR); // viewport X
        const clientPy = Math.round(cy + vy * maxR); // viewport Y (down positive)

        // Prefer programmatic API if available (SetPosition expects client coords)
        if (joystickRef.current && typeof joystickRef.current.SetPosition === "function") {
          if (force > 0) {
            joystickRef.current.SetPosition(clientPx, clientPy);
            pressedRef.current = true;
          } else if (pressedRef.current && typeof joystickRef.current.Release === "function") {
            joystickRef.current.Release();
            pressedRef.current = false;
          }
        } else {
          // Fallback: synthetic mouse events (legacy)
          if (force > 0) {
            if (!pressedRef.current) {
              dispatchMouse(canvas, "mousedown", clientPx, clientPy);
              dispatchMouse(document, "mousemove", clientPx, clientPy);
              pressedRef.current = true;
            } else {
              dispatchMouse(document, "mousemove", clientPx, clientPy);
            }
          } else if (pressedRef.current) {
            dispatchMouse(document, "mouseup", Math.round(cx), Math.round(cy));
            pressedRef.current = false;
          }
        }
      }

      rafRef.current = requestAnimationFrame(computeKeyboardMovement);
    };

    // Keyboard listeners
    const onKeyDown = (e) => {
      const k = e.key;
      const ks = keyStateRef.current;

      if (k === "w" || k === "W") ks.w = true;
      if (k === "a" || k === "A") ks.a = true;
      if (k === "s" || k === "S") ks.s = true;
      if (k === "d" || k === "D") ks.d = true;
      if (k === "ArrowUp") ks.ArrowUp = true;
      if (k === "ArrowLeft") ks.ArrowLeft = true;
      if (k === "ArrowDown") ks.ArrowDown = true;
      if (k === "ArrowRight") ks.ArrowRight = true;

      if (
        ["w", "a", "s", "d", "W", "A", "S", "D",
         "ArrowUp", "ArrowLeft", "ArrowDown", "ArrowRight"].includes(k)
      ) {
        e.preventDefault();
      }
    };

    const onKeyUp = (e) => {
      const k = e.key;
      const ks = keyStateRef.current;

      if (k === "w" || k === "W") ks.w = false;
      if (k === "a" || k === "A") ks.a = false;
      if (k === "s" || k === "S") ks.s = false;
      if (k === "d" || k === "D") ks.d = false;
      if (k === "ArrowUp") ks.ArrowUp = false;
      if (k === "ArrowLeft") ks.ArrowLeft = false;
      if (k === "ArrowDown") ks.ArrowDown = false;
      if (k === "ArrowRight") ks.ArrowRight = false;

      if (
        ["w", "a", "s", "d", "W", "A", "S", "D",
         "ArrowUp", "ArrowLeft", "ArrowDown", "ArrowRight"].includes(k)
      ) {
        e.preventDefault();
      }
    };

    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("keyup", onKeyUp);

    rafRef.current = requestAnimationFrame(computeKeyboardMovement);

    return () => {
      if (pressedRef.current) {
        const canvas = container.querySelector("canvas");
        if (canvas) {
          const rect = canvas.getBoundingClientRect();
          dispatchMouse(
            document,
            "mouseup",
            rect.left + rect.width / 2,
            rect.top + rect.height / 2
          );
        }
      }

      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("keyup", onKeyUp);
      if (rafRef.current) cancelAnimationFrame(rafRef.current);

      if (joystickRef.current?.Destroy) joystickRef.current.Destroy();
      joystickRef.current = null;
    };
  }, [width, height, onMove]);

  return (
    <div
      id="joystick-container"
      ref={containerRef}
      className={className}
      style={{
        width: `${width}px`,
        height: `${height}px`,
        touchAction: "none",
        display: "block",
      }}
    />
  );
};

export default Joystick;
