import React, { createContext, useState, useEffect, useContext } from 'react';

type Language = 'en' | 'ur';

interface LanguageContextType {
  language: Language;
  setLanguage: (language: Language) => void;
  translate: (key: string) => string;
}

const LanguageContext = createContext<LanguageContextType | undefined>(undefined);

export const LanguageProvider = ({ children }) => {
  const [language, setLanguageState] = useState<Language>('en');

  useEffect(() => {
    if (typeof window !== 'undefined') {
      const storedLanguage = localStorage.getItem('lang') as Language;
      if (storedLanguage) {
        setLanguageState(storedLanguage);
      }
    }
  }, []);

  const setLanguage = (lang: Language) => {
    setLanguageState(lang);
    if (typeof window !== 'undefined') {
      localStorage.setItem('lang', lang);
    }
  };

  useEffect(() => {
    if (typeof window !== 'undefined') {
      document.documentElement.dir = language === 'ur' ? 'rtl' : 'ltr';
      document.documentElement.style.fontFamily = language === 'ur' ? 'Arial, Tahoma, sans-serif' : '';
    }
  }, [language]);

  const translations = {
    en: {
      search: 'Search',
      chapters: 'Chapters',
      settings: 'Settings',
      tutorialButton: 'Docusaurus Tutorial - 5min ⏱️',
      homePageTitle: 'Homepage',
      feature1Title: 'Easy to Use',
      feature1Desc: 'Docusaurus was designed from the ground up to be easily installed and used to get your website up and running quickly.',
      feature2Title: 'Focus on What Matters',
      feature2Desc: 'Docusaurus lets you focus on your docs, and we\'ll do the chores. Go ahead and move your docs into the <code>docs</code> directory.',
      feature3Title: 'Powered by React',
      feature3Desc: 'Extend or customize your website layout by reusing React. Docusaurus can be extended while reusing the same header and footer.',
      'Tutorial': 'Tutorial',
      'Blog': 'Blog',
      'Chatbot': 'Chatbot',
      'GitHub': 'GitHub',
    },
    ur: {
      search: 'تلاش کریں',
      chapters: 'ابواب',
      settings: 'ترتیبات',
      tutorialButton: 'ڈوکوسارس ٹیوٹوریل - 5 منٹ ⏱️',
      homePageTitle: 'مرکزی صفحہ',
      feature1Title: 'استعمال میں آسان',
      feature1Desc: 'ڈوکوسارس کو شروع سے ہی اس طرح ڈیزائن کیا گیا تھا کہ اسے آسانی سے انسٹال اور استعمال کیا جا سکے تاکہ آپ کی ویب سائٹ جلدی سے چل سکے۔',
      feature2Title: 'اہم چیزوں پر توجہ مرکوز کریں',
      feature2Desc: 'ڈوکوسaurus آپ کو اپنے دستاویزات پر توجہ مرکوز کرنے دیتا ہے، اور ہم باقی کام کریں گے۔ آگے بڑھیں اور اپنے دستاویزات کو <code>docs</code> ڈائرکٹری میں منتقل کریں۔',
      feature3Title: 'ری ایکٹ کے ذریعے تقویت یافتہ',
      feature3Desc: 'ری ایکٹ کو دوبارہ استعمال کرکے اپنی ویب سائٹ کی ترتیب کو بڑھائیں یا اپنی مرضی کے مطابق بنائیں۔ ڈوکوسارس کو اسی ہیڈر اور فوٹر کو دوبارہ استعمال کرتے ہوئے بڑھایا جاسکتا ہے۔',
      'Tutorial': 'سبق',
      'Blog': 'بلاگ',
      'Chatbot': 'چیٹ بوٹ',
      'GitHub': 'گٹ ہب',
    },
  };

  const translate = (key: string) => {
    return translations[language][key] || key;
  };

  return (
    <LanguageContext.Provider value={{ language, setLanguage, translate }}>
      {children}
    </LanguageContext.Provider>
  );
};

export const useLanguage = () => {
  const context = useContext(LanguageContext);
  if (context === undefined) {
    throw new Error('useLanguage must be used within a LanguageProvider');
  }
  return context;
};
