import React, { createContext, useState, useContext, ReactNode } from 'react';

interface ChatbotContextType {
  selectedText: string | null;
  setSelectedText: (text: string | null) => void;
  isChatbotOpen: boolean;
  setIsChatbotOpen: (isOpen: boolean) => void;
  addChatMessage: (message: { text: string; sender: 'user' | 'bot' }) => void;
  chatMessages: { text: string; sender: 'user' | 'bot' }[];
}

const ChatbotContext = createContext<ChatbotContextType | undefined>(undefined);

export const ChatbotProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const [isChatbotOpen, setIsChatbotOpen] = useState<boolean>(false);
  const [chatMessages, setChatMessages] = useState<{ text: string; sender: 'user' | 'bot' }[]>([]);

  const addChatMessage = (message: { text: string; sender: 'user' | 'bot' }) => {
    setChatMessages((prevMessages) => [...prevMessages, message]);
  };

  return (
    <ChatbotContext.Provider
      value={{
        selectedText,
        setSelectedText,
        isChatbotOpen,
        setIsChatbotOpen,
        addChatMessage,
        chatMessages,
      }}
    >
      {children}
    </ChatbotContext.Provider>
  );
};

export const useChatbot = () => {
  const context = useContext(ChatbotContext);
  if (context === undefined) {
    throw new Error('useChatbot must be used within a ChatbotProvider');
  }
  return context;
};
