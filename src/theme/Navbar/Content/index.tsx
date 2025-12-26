
import React from 'react';
import NavbarContent from '@theme-original/Navbar/Content';
import type ContentType from '@theme/Navbar/Content';
import type {WrapperProps} from '@docusaurus/types';
import LanguageSwitcher from '../items/LanguageSwitcher';

type Props = WrapperProps<typeof ContentType>;

export default function ContentWrapper(props: Props): JSX.Element {
  return (
    <>
      <NavbarContent {...props} />
      <LanguageSwitcher />
    </>
  );
}
