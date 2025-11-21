import { useRef, useEffect } from 'react';

const Joystick = ({ width = 120, height = 120, onMove, className }) => {
  const containerRef = useRef(null);
  const joystickRef = useRef(null);

  useEffect(() => {
    const container = containerRef.current;
    if (!container || !window.JoyStick) return;

    // Remove previous joystick
    if (joystickRef.current && joystickRef.current.Destroy) {
      joystickRef.current.Destroy();
    } else if (container.firstChild) {
      container.removeChild(container.firstChild);
    }

    // Initialize joystick with black colors
    joystickRef.current = new window.JoyStick(
      container.id,
      {
        width,
        height,
        internalFillColor: '#111',    // dark body tone (library gradient uses these)
        internalStrokeColor: '#000',
        externalStrokeColor: '#000',
        internalLineWidth: 3,
        externalLineWidth: 4,
        autoReturnToCenter: true,
      },
      (stickData) => {
        onMove && onMove(stickData);
      }
    );

    // No manual arrow drawing here — library drawExternal/drawInternal render the full appearance.

    // Cleanup on unmount
    return () => {
      if (joystickRef.current && joystickRef.current.Destroy) {
        joystickRef.current.Destroy();
      } else if (container.firstChild) {
        container.removeChild(container.firstChild);
      }
      joystickRef.current = null;
    };
  }, [width, height, onMove]);

  return (
    <div
      id="joystick-container"
      ref={containerRef}
      className={className || undefined}
      style={{
        position: 'fixed',
        right: '20px',
        bottom: '20px',
        width: `${width}px`,
        height: `${height}px`,
        touchAction: 'none',
        zIndex: 1000,
      }}
    />
  );
};

export default Joystick;
