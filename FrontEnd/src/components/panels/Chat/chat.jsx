//  logic

import { useState } from "react";

 // Chat state
  const [chatMessages, setChatMessages] = useState([
    {
      id: 1,
      text: "Hello! I'm your robot assistant. How can I help you today?",
      sender: "robot",
      timestamp: new Date().toISOString(),
      status: "Delivered",
    },
  ]);
  const [chatInput, setChatInput] = useState("");
  const [isRecording, setIsRecording] = useState(false);
  const [isTyping, setIsTyping] = useState(false);
  const chatQuickPrompts = [
    "Provide current mission status",
    "Return to docking station",
    "Begin perimeter scan",
    "Share latest sensor alerts",
  ];

  useEffect(() => {
    if (chatContainerRef.current) {
      chatContainerRef.current.scrollTop =
        chatContainerRef.current.scrollHeight;
    }
  }, [chatMessages]);

    const chatContainerRef = useRef(null);
  

  // Chat functions
  const formatTimestamp = (value) => {
    const date = typeof value === "string" ? new Date(value) : value;
    return date.toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" });
  };

  const handleSendMessage = async (presetText = "") => {
    const messageText = (presetText || chatInput).trim();
    if (!messageText) return;
    const tempId = Date.now();
    const userMessage = {
      id: tempId,
      text: messageText,
      sender: "human",
      timestamp: new Date().toISOString(),
      status: "Sending",
    };
    setChatMessages((prev) => [...prev, userMessage]);
    setChatInput("");

    setIsTyping(true);
    try {
      const data = await requestV1("/chat/messages", {
        method: "POST",
        body: JSON.stringify({ text: messageText }),
      });
      setChatMessages((prev) => {
        const updated = prev.map((msg) =>
          msg.id === tempId
            ? { ...msg, status: data.message?.status || "Delivered" }
            : msg,
        );
        return data.reply ? [...updated, data.reply] : updated;
      });
    } catch (error) {
      console.error("Chat send error", error);
      setChatMessages((prev) =>
        prev.map((msg) =>
          msg.id === tempId ? { ...msg, status: "Failed" } : msg,
        ),
      );
      toast.error(error.message || "Unable to send message");
    } finally {
      setIsTyping(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === "Enter" && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const handleMicClick = () => {
    setIsRecording(!isRecording);
    // TODO: Implement voice recording functionality
  };

  const handleSuggestionClick = (prompt) => {
    handleSendMessage(prompt);
  };

     const loadSettingsAndChat = async () => {
      try {
        const [
          robotRes,
          securityRes,
          securityEventsRes,
          integrationsRes,
          appearanceRes,
          chatRes,
        ] = await Promise.all([
          requestV1("/settings/robot"),
          requestV1("/settings/security"),
          requestV1("/settings/security/events"),
          requestV1("/settings/integrations"),
          requestV1("/settings/appearance"),
          requestV1("/chat/history"),
        ]);

        if (cancelled) {
          return;
        }

        if (robotRes.data) {
          setRobotSettingsState((prev) => ({ ...prev, ...robotRes.data }));
        }
        if (securityRes.data) {
          setSecurityPreferences((prev) => ({ ...prev, ...securityRes.data }));
        }
        if (Array.isArray(securityEventsRes.items)) {
          setSecurityEvents(securityEventsRes.items);
        }
        if (Array.isArray(integrationsRes.items)) {
          setIntegrationItems(integrationsRes.items);
        }
        if (appearanceRes.data?.theme) {
          setSelectedTheme(appearanceRes.data.theme);
        }
        if (Array.isArray(chatRes.items) && chatRes.items.length) {
          setChatMessages(chatRes.items);
        }
      } catch (error) {
        console.error("Failed to load settings/chat data", error);
      }
    };

    loadSettingsAndChat();
    return () => {
      cancelled = true;
    };
  }, [requestV1]);

// RightPane logic 

  // ---------- VOICE RECOGNITION SETUP ----------
  const SpeechRecognition =
    typeof window !== "undefined" &&
    (window.SpeechRecognition || window.webkitSpeechRecognition);
  const recognition = SpeechRecognition ? new SpeechRecognition() : null;
  if (recognition) {
    recognition.continuous = false;
    recognition.interimResults = false;
    recognition.lang = "en-US";
  }

  const handleMicClick = () => {
    if (!recognition) {
      alert("Your browser does not support voice input.");
      return;
    }
    if (isRecording) {
      recognition.stop();
      setIsRecording(false);
      return;
    }
    setIsRecording(true);
    recognition.start();
    recognition.onresult = (event) => {
      const text = event.results[0][0].transcript;
      setChatInput((prev) => prev + (prev ? " " : "") + text);
      setIsRecording(false);
    };
    recognition.onerror = () => setIsRecording(false);
    recognition.onend = () => setIsRecording(false);
  };

  // RightPane logic
    // destructure props (removed isRecording/handleMicClick from props)
  const {
    rightPage,
    setRightPage,
    // chat
    chatMessages,
    chatInput,
    setChatInput,
    handleSendMessage,
    handleKeyPress,
    isTyping,
    handleSuggestionClick,
    chatQuickPrompts,
    chatContainerRef,
    formatTimestamp,
    latestMessage,
  } = props;


//   UI
{/* CHAT */}
{rightPage === "chat" && (
  <div className="chat-pane">

    <div className="chat-header">
      <div>
        <h2>Assistant Link</h2>
        <p>
          Last sync at {latestMessage ? formatTimestamp(latestMessage.timestamp) : "--:--"} ·{" "}
          {chatMessages.filter((m) => m.sender === "human").length} operator prompts today
        </p>
      </div>
      <div className="chat-status-pill">Robot Online</div>
    </div>

    <div className="chat-subheader">
      <span>Channel: Operations Support</span>
      <span>Voice input {isRecording ? "listening…" : "idle"}</span>
    </div>

    <div className="chat-messages" ref={chatContainerRef}>
      {chatMessages.map((message) => {
        const isRobot = message.sender === "robot";
        return (
          <div key={message.id} className={`chat-row ${isRobot ? "robot" : "human"}`}>
            <div className={`chat-bubble ${isRobot ? "robot" : "human"}`}>
              <div className="chat-meta">
                <span>{isRobot ? "Robot" : "You"}</span>
                <span>{formatTimestamp(message.timestamp)}</span>
              </div>
              <p>{message.text}</p>
              {!isRobot && <span className="chat-status">{message.status || "Sent"}</span>}
            </div>
          </div>
        );
      })}

      {isTyping && (
        <div className="chat-row robot">
          <div className="chat-bubble robot typing">
            <span className="typing-dot" />
            <span className="typing-dot" />
            <span className="typing-dot" />
          </div>
        </div>
      )}
    </div>

    <div className="chat-quick-replies">
      {chatQuickPrompts.map((prompt) => (
        <button
          key={prompt}
          type="button"
          className="chat-chip"
          onClick={() => handleSuggestionClick(prompt)}
        >
          {prompt}
        </button>
      ))}
    </div>

    <div className="chat-input-row">
      <textarea
        value={chatInput}
        onChange={(e) => setChatInput(e.target.value)}
        onKeyPress={handleKeyPress}
        placeholder="Type a request or speak…"
        className="chat-textarea"
        rows={1}
      />

      {/* MIC BUTTON UPDATED */}
      <button
        onClick={handleMicClick}
        type="button"
        className={`chat-icon-btn ${isRecording ? "recording" : ""}`}
        title={isRecording ? "Listening…" : "Start voice input"}
      >
        <FaMicrophone />
      </button>

      <button
        onClick={() => handleSendMessage()}
        disabled={!chatInput.trim()}
        type="button"
        className="chat-send-btn"
        title="Send message"
      >
        <FaPaperPlane />
      </button>
    </div>

    <div className="chat-hint">Press Enter to send · Shift + Enter for newline</div>
  </div>
)}