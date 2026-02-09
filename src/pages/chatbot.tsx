import React from 'react';
import Layout from '@theme/Layout';
import Chatbot from '../components/Chatbot';

function ChatbotPage() {
  return (
    <Layout title="Chatbot" description="Chat with the AI about the textbook content.">
      <main style={{
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        padding: '2rem',
        minHeight: 'calc(100vh - var(--ifm-navbar-height) - var(--ifm-footer-height))'
      }}>
        <h1>Textbook Chatbot</h1>
        <p>Ask me anything about the Physical AI & Humanoid Robotics textbook!</p>
        <div style={{ width: '100%', maxWidth: '800px' }}>
          <Chatbot />
        </div>
      </main>
    </Layout>
  );
}

export default ChatbotPage;
