import React, { ReactNode } from 'react';
import { useAuth } from '../../contexts/AuthContext';

interface PersonalizedSectionProps {
  software?: string;
  hardware?: string;
  children: ReactNode;
}

const PersonalizedSection: React.FC<PersonalizedSectionProps> = ({ software, hardware, children }) => {
  const { user } = useAuth();

  if (!user) {
    // If no user is logged in, perhaps show a default or nothing
    return null; 
  }

  const userSoftware = user.software_background?.toLowerCase();
  const userHardware = user.hardware_background?.toLowerCase();

  const matchesSoftware = software ? userSoftware && userSoftware.includes(software.toLowerCase()) : true;
  const matchesHardware = hardware ? userHardware && userHardware.includes(hardware.toLowerCase()) : true;

  if (matchesSoftware && matchesHardware) {
    return <>{children}</>;
  }

  return null;
};

export default PersonalizedSection;
