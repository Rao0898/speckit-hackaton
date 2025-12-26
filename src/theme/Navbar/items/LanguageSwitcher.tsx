
import React from 'react';
import { useLanguage } from '../../../contexts/LanguageContext';

export default function LanguageSwitcher() {
  const { language, setLanguage } = useLanguage();

  const handleLanguageChange = (lang: 'en' | 'ur') => {
    setLanguage(lang);
  };

  return (
    <div style={{ display: 'flex', alignItems: 'center', marginLeft: '1rem' }}>
      <button
        onClick={() => handleLanguageChange('en')}
        style={{
          marginRight: '0.5rem',
          padding: '0.25rem 0.5rem',
          border: language === 'en' ? '2px solid var(--ifm-color-primary)' : '2px solid transparent',
          borderRadius: '4px',
          cursor: 'pointer',
          backgroundColor: 'transparent',
          color: 'var(--ifm-navbar-link-color)',
        }}
      >
        EN
      </button>
      <button
        onClick={() => handleLanguageChange('ur')}
        style={{
          padding: '0.25rem 0.5rem',
          border: language === 'ur' ? '2px solid var(--ifm-color-primary)' : '2px solid transparent',
          borderRadius: '4px',
          cursor: 'pointer',
          backgroundColor: 'transparent',
          color: 'var(--ifm-navbar-link-color)',
        }}
      >
        UR
      </button>
    </div>
  );
}
