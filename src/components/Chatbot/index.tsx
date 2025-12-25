import React, { useState, FormEvent, useEffect, useRef } from 'react';
import styles from './styles.module.css';
import { useChatbot } from '../../contexts/ChatbotContext';

interface Message {
  text: string;
  sender: 'user' | 'bot';
}

interface ChatbotProps {
  initialMessages: Message[];
  onNewMessage: (message: Message) => void;
}

const Chatbot: React.FC<ChatbotProps> = ({ initialMessages, onNewMessage }) => {
  const { selectedText, chatMessages, addChatMessage } = useChatbot();
  const [input, setInput] = useState<string>('');
  const [loading, setLoading] = useState<boolean>(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Fix #1: Safe variables for length check
  const safeInitial = initialMessages || [];
  const safeChat = chatMessages || [];

  useEffect(() => {
    if (safeInitial.length !== safeChat.length || safeInitial[safeInitial.length - 1] !== safeChat[safeChat.length - 1]) {
      // Sync logic if needed
    }
  }, [safeInitial, safeChat]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };
  useEffect(scrollToBottom, [chatMessages]);

  const handleSendMessage = async (e: FormEvent) => {
    e.preventDefault();
    if (!input.trim() && !selectedText) return;

    let currentMessage = input.trim();
    if (selectedText && !input.trim()) {
      // Fix #2: selectedText safe length check
      currentMessage = `Based on "${selectedText}", ${selectedText && selectedText.length > 50 ? "what can you tell me?" : "explain this."}`;
    } else if (selectedText) {
      currentMessage = `Regarding "${selectedText}", ${input.trim()}`;
    }

    const userMessage: Message = { text: currentMessage, sender: 'user' };
    addChatMessage(userMessage);
    setInput('');
    setLoading(true);

    try {
      let endpoint = 'http://localhost:8000/api/v1/query';
      let requestBody: any = { query: currentMessage };

      if (selectedText) {
        endpoint = 'http://localhost:8000/api/v1/query-selection';
        requestBody = { query: currentMessage, selection: selectedText };
      }

      // MOCK IMPLEMENTATION FOR DEMONSTRATION
      const mockFetch = (endpoint: string, body: any) => {
        console.log("Mock API Call to:", endpoint);
        console.log("Request Body:", body);
        return new Promise((resolve) => {
          setTimeout(() => {
            const responseData = {
              answer: "This is a mocked answer to your question: '" + (body.query || "") + "'. The integration is working correctly.",
              citations: [
                { source_file: "Chapter 1: Introduction", text: "Robotics is an interdisciplinary branch..." }
              ]
            };
            resolve({
              json: () => Promise.resolve(responseData),
            });
          }, 1000);
        });
      };

      const response: any = await mockFetch(endpoint, requestBody);
      // END MOCK IMPLEMENTATION

      const data = await response.json();
      const botMessage: Message = { text: data?.answer || "Sorry, I couldn't get an answer.", sender: 'bot' };
      addChatMessage(botMessage);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage: Message = { text: 'An error occurred. Please try again.', sender: 'bot' };
      addChatMessage(errorMessage);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className={styles.chatbotContainer}>
      <div className={styles.messagesContainer}>
        {(chatMessages || []).map((msg, index) => (
          <div key={index} className={`${styles.message} ${styles[msg.sender]}`}>
            {msg.text}
          </div>
        ))}
        {loading && <div className={`${styles.message} ${styles.bot}`}>Loading...</div>}
        <div ref={messagesEndRef} />
      </div>

      {selectedText && (
        <div className={styles.selectedTextDisplay} onClick={() => setInput(selectedText)}>
          Context: "{selectedText && (selectedText.length > 100 ? selectedText.substring(0, 100) + '...' : selectedText)}"
        </div>
      )}

      <form onSubmit={handleSendMessage} className={styles.inputForm}>
        <input
          type="text"
          value={input}
          onChange={(e) => setInput(e.target.value)}
          placeholder={selectedText ? `Ask about "${selectedText?.substring(0, 20)}..."` : "Ask a question about the textbook..."}
          className={styles.textInput}
          disabled={loading}
        />
        <button type="submit" disabled={loading} className={styles.sendButton}>
          Send
        </button>
      </form>
    </div>
  );
};

export default Chatbot;
