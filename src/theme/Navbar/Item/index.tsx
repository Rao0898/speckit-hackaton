
import React from 'react';
import NavbarItem from '@theme-original/Navbar/Item';
import type { Props } from '@theme/Navbar/Item';
import { useLanguage } from '../../../contexts/LanguageContext';

export default function ItemWrapper(props: Props): JSX.Element {
  const { translate } = useLanguage();
  const { label } = props;

  if (label) {
    const translatedLabel = translate(label);
    return <NavbarItem {...props} label={translatedLabel} />;
  }

  return <NavbarItem {...props} />;
}
