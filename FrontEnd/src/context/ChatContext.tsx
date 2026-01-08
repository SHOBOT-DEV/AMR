import React, { createContext, useContext, useState } from "react";

const SAMPLE_CHAT_MESSAGES = [
    { id: 1, text: "Hello! I'm your robot assistant. How can I help you today?", sender: "robot", timestamp: new Date().toISOString(), status: "Delivered" },
];

const CHAT_QUICK_PROMPTS = [
    "Provide current mission status",
    "Return to docking station",
    "Begin perimeter scan",
    "Share latest sensor alerts",
];

type ChatContextShape = {
    chatMessages: any[];
    setChatMessages: React.Dispatch<React.SetStateAction<any[]>>;
    chatQuickPrompts: string[];
};

const ChatContext = createContext<ChatContextShape | undefined>(undefined);

export const ChatProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
    const [chatMessages, setChatMessages] = useState<any[]>(SAMPLE_CHAT_MESSAGES);
    return (
        <ChatContext.Provider value={{ chatMessages, setChatMessages, chatQuickPrompts: CHAT_QUICK_PROMPTS }}>
            {children}
        </ChatContext.Provider>
    );
};

export const useChat = (): ChatContextShape => {
    const ctx = useContext(ChatContext);
    if (!ctx) throw new Error("useChat must be used within ChatProvider");
    return ctx;
};