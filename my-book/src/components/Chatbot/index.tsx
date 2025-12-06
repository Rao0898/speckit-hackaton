import React, { useState } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './styles.module.css';

interface ChatMessage {
  text: string;
  sender: 'user' | 'bot';
  sourceChunks?: any[]; // To store source document chunks
}

function Chatbot() {
  const { siteConfig } = useDocusaurusContext();
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [input, setInput] = useState('');
  const [loading, setLoading] = useState(false);

  const sendMessage = async () => {
    if (!input.trim()) return;

    const userMessage: ChatMessage = { text: input, sender: 'user' };
    setMessages((prev) => [...prev, userMessage]);
    setInput('');
    setLoading(true);

    try {
      // Assuming your FastAPI backend is running on http://localhost:8000
      const response = await fetch('http://localhost:8000/rag-answer', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ query: userMessage.text }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      const botMessage: ChatMessage = {
        text: data.answer || 'I could not find an answer.',
        sender: 'bot',
        sourceChunks: data.source_chunks || [],
      };
      setMessages((prev) => [...prev, botMessage]);
    } catch (error) {
      console.error('Chatbot API error:', error);
      setMessages((prev) => [
        ...prev,
        { text: 'Oops! Something went wrong. Please try again.', sender: 'bot' },
      ]);
    } finally {
      setLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter') {
      sendMessage();
    }
  };

  return (
    <div className={styles.chatbotContainer}>
      <div className={styles.messagesContainer}>
        {messages.map((msg, index) => (
          <div key={index} className={`${styles.message} ${styles[msg.sender]}`}>
            <p>{msg.text}</p>
            {msg.sender === 'bot' && msg.sourceChunks && msg.sourceChunks.length > 0 && (
              <div className={styles.sourceChunks}>
                <h4>Sources:</h4>
                <ul>
                  {msg.sourceChunks.map((chunk, idx) => (
                    <li key={idx}>
                      <a href={`/docs/${chunk.document_id.replace(/\.mdx?$/, '')}`} target="_blank" rel="noopener noreferrer">
                        {chunk.metadata?.chapter} - {chunk.metadata?.section} (Chunk ID: {chunk.chunk_id.substring(0, 8)})
                      </a>
                    </li>
                  ))}
                </ul>
              </div>
            )}
          </div>
        ))}
        {loading && <div className={styles.loadingMessage}>Thinking...</div>}
      </div>
      <div className={styles.inputContainer}>
        <input
          type="text"
          value={input}
          onChange={(e) => setInput(e.target.value)}
          onKeyPress={handleKeyPress}
          placeholder="Ask me a question about the textbook..."
          disabled={loading}
        />
        <button onClick={sendMessage} disabled={loading}>
          Send
        </button>
      </div>
    </div>
  );
}

export default Chatbot;
