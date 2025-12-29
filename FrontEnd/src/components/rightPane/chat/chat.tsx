import React, { useState } from "react";
import {
  FaMicrophone,
  FaPaperPlane,
} from "react-icons/fa";

type ChatProps = {
  chatMessages: any[];
  chatInput: string;
  setChatInput: (v: string | ((p: string) => string)) => void;
  handleSendMessage: () => void;
  handleKeyPress: (e: React.KeyboardEvent<HTMLTextAreaElement>) => void;
  isTyping: boolean;
  handleSuggestionClick: (p: string) => void;
  chatQuickPrompts: string[];
  chatContainerRef: React.RefObject<HTMLDivElement>;
  formatTimestamp: (t: any) => string;
  latestMessage: any;
};

const Chat: React.FC<ChatProps> = ({
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
}) => {
  const [isRecording, setIsRecording] = useState(false);

  // 🎤 Voice recognition
  const handleMicClick = () => {
    const SpeechRecognition =
      window.SpeechRecognition || (window as any).webkitSpeechRecognition;

    if (!SpeechRecognition) {
      alert("Your browser does not support voice input.");
      return;
    }

    const recognition = new SpeechRecognition();
    recognition.continuous = false;
    recognition.interimResults = false;
    recognition.lang = "en-US";

    if (isRecording) {
      recognition.stop();
      setIsRecording(false);
      return;
    }

    setIsRecording(true);
    recognition.start();

    recognition.onresult = (event: any) => {
      const text = event.results[0][0].transcript;
      setChatInput((prev) => prev + (prev ? " " : "") + text);
      setIsRecording(false);
    };

    recognition.onerror = () => setIsRecording(false);
    recognition.onend = () => setIsRecording(false);
  };

  return (
    <div className="flex flex-col h-full bg-white">
      {/* Header */}
      <div className="flex-none p-4 bg-slate-50 border-b border-slate-100">
        <div className="flex justify-between items-start">
          <div>
            <h2 className="text-lg font-bold text-slate-800">
              Assistant Link
            </h2>
            <p className="text-xs text-slate-500 mt-0.5">
              Last sync at{" "}
              {latestMessage ? formatTimestamp(latestMessage.timestamp) : "--:--"}
            </p>
          </div>
          <span className="px-2 py-1 bg-emerald-500 text-white text-xs font-bold rounded-full">
            Online
          </span>
        </div>
      </div>

      {/* Messages */}
      <div
        className="flex-1 overflow-y-auto p-4 space-y-4"
        ref={chatContainerRef}
      >
        {chatMessages.map((msg) => {
          const isRobot = msg.sender === "robot";
          return (
            <div
              key={msg.id}
              className={`flex w-full ${
                isRobot ? "justify-start" : "justify-end"
              }`}
            >
              <div
                className={`max-w-[80%] p-3 rounded-2xl text-sm shadow-sm ${
                  isRobot
                    ? "bg-slate-100 text-slate-800 rounded-bl-sm"
                    : "bg-sky-600 text-white rounded-br-sm"
                }`}
              >
                <p>{msg.text}</p>
                <div
                  className={`text-[10px] mt-1 opacity-70 ${
                    isRobot
                      ? "text-slate-500"
                      : "text-sky-100 text-right"
                  }`}
                >
                  {formatTimestamp(msg.timestamp)}
                </div>
              </div>
            </div>
          );
        })}

        {isTyping && (
          <div className="flex justify-start">
            <div className="bg-slate-100 p-3 rounded-2xl rounded-bl-sm flex gap-1">
              <div className="w-2 h-2 bg-slate-400 rounded-full animate-bounce"></div>
              <div className="w-2 h-2 bg-slate-400 rounded-full animate-bounce delay-75"></div>
              <div className="w-2 h-2 bg-slate-400 rounded-full animate-bounce delay-150"></div>
            </div>
          </div>
        )}
      </div>

      {/* Input */}
      <div className="flex-none p-4 border-t border-slate-100 bg-white">
        <div className="flex gap-2 overflow-x-auto pb-2 mb-2">
          {chatQuickPrompts.map((p) => (
            <button
              key={p}
              onClick={() => handleSuggestionClick(p)}
              className="flex-none px-3 py-1 bg-slate-50 border border-slate-200 rounded-full text-xs text-slate-600 hover:bg-sky-50 hover:text-sky-600 hover:border-sky-200 transition"
            >
              {p}
            </button>
          ))}
        </div>

        <div className="flex gap-2 items-end">
          <textarea
            value={chatInput}
            onChange={(e) => setChatInput(e.target.value)}
            onKeyPress={handleKeyPress}
            placeholder="Type a message..."
            className="flex-1 bg-slate-50 border border-slate-200 rounded-xl px-4 py-3 text-sm focus:ring-2 focus:ring-sky-500 resize-none max-h-32"
            rows={1}
          />

          <button
            onClick={handleMicClick}
            className={`p-3 rounded-xl transition-colors ${
              isRecording
                ? "bg-rose-500 text-white animate-pulse"
                : "bg-slate-100 text-slate-500 hover:bg-slate-200"
            }`}
          >
            <FaMicrophone />
          </button>

          <button
            onClick={handleSendMessage}
            disabled={!chatInput.trim()}
            className="p-3 bg-sky-600 text-white rounded-xl hover:bg-sky-700 disabled:bg-slate-200 disabled:text-slate-400 transition shadow-sm"
          >
            <FaPaperPlane />
          </button>
        </div>
      </div>
    </div>
  );
};

export default Chat;
