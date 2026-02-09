import React, { type ReactNode, useEffect } from 'react';
import Layout from '@theme-original/Layout';
import type LayoutType from '@theme/Layout';
import type {WrapperProps} from '@docusaurus/types';
import { ChatbotProvider, useChatbot } from '../../contexts/ChatbotContext';
import Chatbot from '../../components/Chatbot';

type Props = WrapperProps<typeof LayoutType>;

function LayoutContent(props: Props): ReactNode {
  const { setSelectedText, setIsChatbotOpen, isChatbotOpen, chatMessages, addChatMessage } = useChatbot();

  useEffect(() => {
    const handleMouseUp = () => {
      const selection = window.getSelection();
      if (selection && selection.toString().length > 0) {
        setSelectedText(selection.toString());
      } else {
        setSelectedText(null);
      }
    };

    document.addEventListener('mouseup', handleMouseUp);
    document.addEventListener('touchend', handleMouseUp);

    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
      document.removeEventListener('touchend', handleMouseUp);
    };
  }, [setSelectedText]);

  const handleToggleChatbot = () => {
    setIsChatbotOpen(!isChatbotOpen);
  };

  return (
    <>
      <Layout {...props} />
      <button
        onClick={handleToggleChatbot}
        style={{
          position: 'fixed',
          bottom: '20px',
          right: '20px',
          zIndex: 1000,
          backgroundColor: 'var(--ifm-color-primary)',
          color: 'white',
          border: 'none',
          borderRadius: '50%',
          width: '50px',
          height: '50px',
          fontSize: '24px',
          cursor: 'pointer',
          boxShadow: '0 2px 10px rgba(0,0,0,0.2)',
        }}
      >
        {isChatbotOpen ? 'X' : '💬'}
      </button>
      {isChatbotOpen && (
        <div
          style={{
            position: 'fixed',
            bottom: '90px',
            right: '20px',
            zIndex: 1000,
            width: '400px', // Adjust width as needed
            height: '600px', // Adjust height as needed
            boxShadow: '0 5px 15px rgba(0,0,0,0.3)',
            borderRadius: '10px',
            overflow: 'hidden',
          }}
        >
          <Chatbot initialMessages={chatMessages} onNewMessage={addChatMessage} />
        </div>
      )}
    </>
  );
}

export default function LayoutWrapper(props: Props): ReactNode {
  return (
    <ChatbotProvider>
     
        <LayoutContent {...props} />
      
    </ChatbotProvider>
  );
}
