let StickStatus =
{
    xPosition: 0,
    yPosition: 0,
    x: 0,
    y: 0,
    cardinalDirection: "C"
};

// Define JoyStick as a proper constructor function
function JoyStick(container, parameters, callback)
{
    parameters = parameters || {};
    var title = (typeof parameters.title === "undefined" ? "joystick" : parameters.title);
    var width = (typeof parameters.width === "undefined" ? 0 : parameters.width);
    var height = (typeof parameters.height === "undefined" ? 0 : parameters.height);

    // default colors (black)
    var internalFillColor = (typeof parameters.internalFillColor === "undefined" ? "#000" : parameters.internalFillColor);
    var internalLineWidth = (typeof parameters.internalLineWidth === "undefined" ? 2 : parameters.internalLineWidth);
    var internalStrokeColor = (typeof parameters.internalStrokeColor === "undefined" ? "#000" : parameters.internalStrokeColor);
    var externalLineWidth = (typeof parameters.externalLineWidth === "undefined" ? 2 : parameters.externalLineWidth);
    var externalStrokeColor = (typeof parameters.externalStrokeColor ===  "undefined" ? "#000" : parameters.externalStrokeColor);
    var autoReturnToCenter = (typeof parameters.autoReturnToCenter === "undefined" ? true : parameters.autoReturnToCenter);

    callback = callback || function(StickStatus) {};

    // Create Canvas element and add it in the Container object
    var objContainer = document.getElementById(container);
    if (!objContainer) return;

    // Fixing passive listener issue
    objContainer.style.touchAction = "none";

    var canvas = document.createElement("canvas");
    canvas.id = title;
    if(width === 0) { width = objContainer.clientWidth; }
    if(height === 0) { height = objContainer.clientHeight; }
    canvas.width = width;
    canvas.height = height;
    objContainer.appendChild(canvas);
    var context = canvas.getContext("2d");

    var pressed = 0; // Bool - 1=Yes - 0=No
    var circumference = 2 * Math.PI;
    var internalRadius = (canvas.width - ((canvas.width / 2) + 10)) / 2;
    var maxMoveStick = internalRadius + 5;
    var externalRadius = internalRadius + 30;
    var centerX = canvas.width / 2;
    var centerY = canvas.height / 2;
    var directionHorizontalLimitPos = canvas.width / 10;
    var directionHorizontalLimitNeg = directionHorizontalLimitPos * -1;
    var directionVerticalLimitPos = canvas.height / 10;
    var directionVerticalLimitNeg = directionVerticalLimitPos * -1;
    // Used to save current position of stick
    var movedX = centerX;
    var movedY = centerY;

    // Event handler references (so we can remove them in Destroy)
    var onTouchStartRef, onTouchMoveRef, onTouchEndRef, onMouseDownRef, onMouseMoveRef, onMouseUpRef;

    // Check if the device support the touch or not
    if("ontouchstart" in document.documentElement)
    {
        onTouchStartRef = onTouchStart;
        onTouchMoveRef = onTouchMove;
        onTouchEndRef = onTouchEnd;
        canvas.addEventListener("touchstart", onTouchStartRef, false);
        document.addEventListener("touchmove", onTouchMoveRef, false);
        document.addEventListener("touchend", onTouchEndRef, false);
    }
    else
    {
        onMouseDownRef = onMouseDown;
        onMouseMoveRef = onMouseMove;
        onMouseUpRef = onMouseUp;
        canvas.addEventListener("mousedown", onMouseDownRef, false);
        document.addEventListener("mousemove", onMouseMoveRef, false);
        document.addEventListener("mouseup", onMouseUpRef, false);
    }
    // Draw the object
    drawExternal();
    drawInternal();

    /******************************************************
     * Private methods
     *****************************************************/

    /**
     * @desc Draw the external circle used as reference position
     */
    function drawExternal()
    {
        // Draw small separators/notches (very subtle darker spots) around where the ring would be
        var notchCount = 8;
        for (var i = 0; i < notchCount; i++) {
            var angle = (i / notchCount) * circumference;
            var nx = centerX + Math.cos(angle) * (externalRadius - 4);
            var ny = centerY + Math.sin(angle) * (externalRadius - 4);
            context.beginPath();
            context.arc(nx, ny, 1.2, 0, circumference, false);
            context.fillStyle = "#6b6b6b";
            context.fill();
        }

        // Draw four small directional carets (arrowheads) closely outside the inner ring
        var arrowSize = Math.max(8, Math.floor(externalRadius * 0.08)); // small triangles
        context.fillStyle = "#111"; // almost black

        var offset = externalRadius - 2; // place carets just at inner-ring edge

        // Up caret
        context.beginPath();
        context.moveTo(centerX, centerY - offset - (arrowSize + 2));
        context.lineTo(centerX - arrowSize / 2, centerY - offset + 2);
        context.lineTo(centerX + arrowSize / 2, centerY - offset + 2);
        context.closePath();
        context.fill();

        // Right caret
        context.beginPath();
        context.moveTo(centerX + offset + (arrowSize + 2), centerY);
        context.lineTo(centerX + offset - 2, centerY - arrowSize / 2);
        context.lineTo(centerX + offset - 2, centerY + arrowSize / 2);
        context.closePath();
        context.fill();

        // Down caret
        context.beginPath();
        context.moveTo(centerX, centerY + offset + (arrowSize + 2));
        context.lineTo(centerX - arrowSize / 2, centerY + offset - 2);
        context.lineTo(centerX + arrowSize / 2, centerY + offset - 2);
        context.closePath();
        context.fill();

        // Left caret
        context.beginPath();
        context.moveTo(centerX - offset - (arrowSize + 2), centerY);
        context.lineTo(centerX - offset + 2, centerY - arrowSize / 2);
        context.lineTo(centerX - offset + 2, centerY + arrowSize / 2);
        context.closePath();
        context.fill();

        // Draw the outer circle outline AFTER arrows — no fill, very subtle stroke (almost invisible)
        context.beginPath();
        context.arc(centerX, centerY, externalRadius, 0, circumference, false);
        context.lineWidth = 1; // thin outline
        context.strokeStyle = "rgba(0,0,0,0.06)"; // nearly transparent; change alpha if you want it more/less visible
        context.stroke();
    }

    /**
     * @desc Draw the internal stick in the current position the user have moved it
     */
    function drawInternal()
    {
        // Safety bounds (keep same behavior)
        if(movedX < internalRadius) { movedX = maxMoveStick; }
        if((movedX + internalRadius) > canvas.width) { movedX = canvas.width - (maxMoveStick); }
        if(movedY < internalRadius) { movedY = maxMoveStick; }
        if((movedY + internalRadius) > canvas.height) { movedY = canvas.height - (maxMoveStick); }

        // Outer dark rim for the knob (gives ring separation)
        context.beginPath();
        context.arc(movedX, movedY, internalRadius + 6, 0, circumference, false);
        context.fillStyle = "#4a4a4a";
        context.fill();

        // Knob main body: radial gradient for glossy black/dark gray look
        context.beginPath();
        var knobRadius = Math.max( (internalRadius - 8), 12 );
        context.arc(movedX, movedY, knobRadius, 0, circumference, false);
        var grd = context.createRadialGradient(
            movedX - knobRadius * 0.25,
            movedY - knobRadius * 0.25,
            Math.max(2, knobRadius * 0.05),
            movedX,
            movedY,
            knobRadius * 1.1
        );
        grd.addColorStop(0, "#7a7a7a");   // subtle highlight (top-left)
        grd.addColorStop(0.5, "#2b2b2b"); // mid tone
        grd.addColorStop(1, "#0a0a0a");   // edge (near black)
        context.fillStyle = grd;
        context.fill();

        // Slight inner glossy sheen (top-left)
        context.beginPath();
        var sheenRadius = Math.max(6, Math.floor(knobRadius * 0.5));
        context.ellipse(movedX - knobRadius * 0.25, movedY - knobRadius * 0.25, sheenRadius, sheenRadius * 0.7, -0.3, 0, Math.PI * 2);
        var sheenGrd = context.createLinearGradient(movedX - knobRadius, movedY - knobRadius, movedX + knobRadius, movedY + knobRadius);
        sheenGrd.addColorStop(0, "rgba(255,255,255,0.6)");
        sheenGrd.addColorStop(1, "rgba(255,255,255,0.0)");
        context.fillStyle = sheenGrd;
        context.fill();

        // Small central disc (bright-ish) to mimic the inner button
        var innerBtnRadius = Math.max(10, Math.floor(knobRadius * 0.38));
        context.beginPath();
        context.arc(movedX, movedY, innerBtnRadius, 0, circumference, false);
        var btnGrd = context.createRadialGradient(
            movedX - innerBtnRadius * 0.2,
            movedY - innerBtnRadius * 0.2,
            1,
            movedX,
            movedY,
            innerBtnRadius * 1.2
        );
        btnGrd.addColorStop(0, "#f0f0f0");
        btnGrd.addColorStop(0.6, "#6b6b6b");
        btnGrd.addColorStop(1, "#111111");
        context.fillStyle = btnGrd;
        context.fill();

        // Rim around the inner disc for definition
        context.lineWidth = 1.6;
        context.strokeStyle = "#000";
        context.stroke();

        // Center label "Auto" in white (matches screenshot)
        context.fillStyle = "#ffffff";
        var fontSize = Math.max(10, Math.floor(innerBtnRadius * 0.6));
        context.font = fontSize + "px Arial";
        context.textAlign = "center";
        context.textBaseline = "middle";
        context.fillText("Auto", movedX, movedY + 1); // small optical adjust
    }

    /**
     * @desc Events for manage touch
     */
    let touchId = null;
    function onTouchStart(event)
    {
        pressed = 1;
        touchId = event.targetTouches[0].identifier;
    }

    function onTouchMove(event)
    {
        if(pressed === 1 && event.targetTouches[0].target === canvas)
        {
            movedX = event.targetTouches[0].pageX;
            movedY = event.targetTouches[0].pageY;
            // Manage offset
            if(canvas.offsetParent && canvas.offsetParent.tagName && canvas.offsetParent.tagName.toUpperCase() === "BODY")
            {
                movedX -= canvas.offsetLeft;
                movedY -= canvas.offsetTop;
            }
            else if (canvas.offsetParent) {
                movedX -= canvas.offsetParent.offsetLeft;
                movedY -= canvas.offsetParent.offsetTop;
            }
            // Delete canvas
            context.clearRect(0, 0, canvas.width, canvas.height);
            // Redraw object
            drawExternal();
            drawInternal();

            // Set attribute of callback
            StickStatus.xPosition = movedX;
            StickStatus.yPosition = movedY;
            StickStatus.x = (100 * ((movedX - centerX) / maxMoveStick)).toFixed();
            StickStatus.y = ((100 * ((movedY - centerY) / maxMoveStick)) * -1).toFixed();
            StickStatus.cardinalDirection = getCardinalDirection();
            callback(StickStatus);
        }
    }

    function onTouchEnd(event)
    {
        // If touchId doesn't match changed touch, ignore
        var changed = event.changedTouches ? event.changedTouches[0] : null;
        if (changed && changed.identifier !== touchId) return;

        pressed = 0;
        // If required reset position store variable
        if(autoReturnToCenter)
        {
            movedX = centerX;
            movedY = centerY;
        }
        // Delete canvas
        context.clearRect(0, 0, canvas.width, canvas.height);
        // Redraw object
        drawExternal();
        drawInternal();

        // Set attribute of callback
        StickStatus.xPosition = movedX;
        StickStatus.yPosition = movedY;
        StickStatus.x = (100 * ((movedX - centerX) / maxMoveStick)).toFixed();
        StickStatus.y = ((100 * ((movedY - centerY) / maxMoveStick)) * -1).toFixed();
        StickStatus.cardinalDirection = getCardinalDirection();
        callback(StickStatus);
    }

    /**
     * @desc Events for manage mouse
     */
    function onMouseDown(event)
    {
        pressed = 1;
    }

    /* To simplify this code there was a new experimental feature here: https://developer.mozilla.org/en-US/docs/Web/API/MouseEvent/offsetX , but it present only in Mouse case not metod presents in Touch case :-( */
    function onMouseMove(event)
    {
        if(pressed === 1)
        {
            movedX = event.pageX;
            movedY = event.pageY;
            // Manage offset
            if(canvas.offsetParent && canvas.offsetParent.tagName && canvas.offsetParent.tagName.toUpperCase() === "BODY")
            {
                movedX -= canvas.offsetLeft;
                movedY -= canvas.offsetTop;
            }
            else if (canvas.offsetParent)
            {
                movedX -= canvas.offsetParent.offsetLeft;
                movedY -= canvas.offsetParent.offsetTop;
            }
            // Delete canvas
            context.clearRect(0, 0, canvas.width, canvas.height);
            // Redraw object
            drawExternal();
            drawInternal();

            // Set attribute of callback
            StickStatus.xPosition = movedX;
            StickStatus.yPosition = movedY;
            StickStatus.x = (100 * ((movedX - centerX) / maxMoveStick)).toFixed();
            StickStatus.y = ((100 * ((movedY - centerY) / maxMoveStick)) * -1).toFixed();
            StickStatus.cardinalDirection = getCardinalDirection();
            callback(StickStatus);
        }
    }

    function onMouseUp(event)
    {
        pressed = 0;
        // If required reset position store variable
        if(autoReturnToCenter)
        {
            movedX = centerX;
            movedY = centerY;
        }
        // Delete canvas
        context.clearRect(0, 0, canvas.width, canvas.height);
        // Redraw object
        drawExternal();
        drawInternal();

        // Set attribute of callback
        StickStatus.xPosition = movedX;
        StickStatus.yPosition = movedY;
        StickStatus.x = (100 * ((movedX - centerX) / maxMoveStick)).toFixed();
        StickStatus.y = ((100 * ((movedY - centerY) / maxMoveStick)) * -1).toFixed();
        StickStatus.cardinalDirection = getCardinalDirection();
        callback(StickStatus);
    }

    function getCardinalDirection()
    {
        var result = "";
        var orizontal = movedX - centerX;
        var vertical = movedY - centerY;

        if(vertical >= directionVerticalLimitNeg && vertical <= directionVerticalLimitPos)
        {
            result = "C";
        }
        if(vertical < directionVerticalLimitNeg)
        {
            result = "N";
        }
        if(vertical > directionVerticalLimitPos)
        {
            result = "S";
        }

        if(orizontal < directionHorizontalLimitNeg)
        {
            if(result === "C")
            {
                result = "W";
            }
            else
            {
                result += "W";
            }
        }
        if(orizontal > directionHorizontalLimitPos)
        {
            if(result === "C")
            {
                result = "E";
            }
            else
            {
                result += "E";
            }
        }

        return result;
    }

    /******************************************************
     * Public methods
     *****************************************************/

    this.GetWidth = function ()
    {
        return canvas.width;
    };

    this.GetHeight = function ()
    {
        return canvas.height;
    };

    this.GetPosX = function ()
    {
        return movedX;
    };

    this.GetPosY = function ()
    {
        return movedY;
    };

    this.GetX = function ()
    {
        return (100 * ((movedX - centerX) / maxMoveStick)).toFixed();
    };

    this.GetY = function ()
    {
        return ((100 * ((movedY - centerY) / maxMoveStick)) * -1).toFixed();
    };

    this.GetDir = function()
    {
        return getCardinalDirection();
    };

    /**
     * Programmatically set joystick position (client coordinates - viewport).
     * This updates visuals and invokes the instance callback, same as a real pointer move.
     */
    this.SetPosition = function(clientX, clientY)
    {
        try {
            pressed = 1;
            // Convert client (viewport) coordinates to canvas-local coordinates using bounding rect
            var rect = canvas.getBoundingClientRect();
            // clientX/clientY are viewport coords; movedX/movedY must be canvas-local
            movedX = clientX - rect.left;
            movedY = clientY - rect.top;

            // Safety: clamp moved values to canvas bounds
            if (movedX < 0) movedX = 0;
            if (movedY < 0) movedY = 0;
            if (movedX > canvas.width) movedX = canvas.width;
            if (movedY > canvas.height) movedY = canvas.height;

            // redraw
            context.clearRect(0, 0, canvas.width, canvas.height);
            drawExternal();
            drawInternal();

            // update status and callback
            StickStatus.xPosition = movedX;
            StickStatus.yPosition = movedY;
            StickStatus.x = (100 * ((movedX - centerX) / maxMoveStick)).toFixed();
            StickStatus.y = ((100 * ((movedY - centerY) / maxMoveStick)) * -1).toFixed();
            StickStatus.cardinalDirection = getCardinalDirection();
            callback(StickStatus);
        } catch (e) {
            // ignore
        }
    };

    /**
     * Programmatically release the joystick (mouseup equivalent).
     */
    this.Release = function()
    {
        try {
            pressed = 0;
            if(autoReturnToCenter)
            {
                movedX = centerX;
                movedY = centerY;
            }
            context.clearRect(0, 0, canvas.width, canvas.height);
            drawExternal();
            drawInternal();
            StickStatus.xPosition = movedX;
            StickStatus.yPosition = movedY;
            StickStatus.x = (100 * ((movedX - centerX) / maxMoveStick)).toFixed();
            StickStatus.y = ((100 * ((movedY - centerY) / maxMoveStick)) * -1).toFixed();
            StickStatus.cardinalDirection = getCardinalDirection();
            callback(StickStatus);
        } catch (e) {
            // ignore
        }
    };

    // Simple Destroy method to allow wrapper cleanup
    this.Destroy = function()
    {
        try {
            if("ontouchstart" in document.documentElement)
            {
                if(onTouchStartRef) canvas.removeEventListener("touchstart", onTouchStartRef, false);
                if(onTouchMoveRef) document.removeEventListener("touchmove", onTouchMoveRef, false);
                if(onTouchEndRef) document.removeEventListener("touchend", onTouchEndRef, false);
            }
            else
            {
                if(onMouseDownRef) canvas.removeEventListener("mousedown", onMouseDownRef, false);
                if(onMouseMoveRef) document.removeEventListener("mousemove", onMouseMoveRef, false);
                if(onMouseUpRef) document.removeEventListener("mouseup", onMouseUpRef, false);
            }
            if (canvas && canvas.parentNode) canvas.parentNode.removeChild(canvas);
        } catch (e) {
            // ignore cleanup errors
        }
    };
}