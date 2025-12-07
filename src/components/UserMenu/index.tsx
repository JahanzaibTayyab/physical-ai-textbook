/**
 * User menu component for navbar.
 *
 * Shows user info and logout when logged in, or sign in/sign up links when not logged in.
 */

import React, { useEffect, useState } from "react";

import styles from "./UserMenu.module.css";

export default function UserMenu(): React.JSX.Element {
  const [userId, setUserId] = useState<string | null>(null);
  const [userEmail, setUserEmail] = useState<string | null>(null);
  const [isOpen, setIsOpen] = useState(false);

  useEffect(() => {
    const storedUserId = localStorage.getItem("user_id");
    const storedEmail = localStorage.getItem("user_email");
    setUserId(storedUserId);
    setUserEmail(storedEmail);
  }, []);

  const handleLogout = () => {
    localStorage.removeItem("session_token");
    localStorage.removeItem("user_id");
    localStorage.removeItem("user_email");
    window.location.href = "/";
  };

  if (!userId) {
    return (
      <div className={styles.userMenu}>
        <a href="/signin" className={styles.signInLink}>
          Sign In
        </a>
        <a href="/signup" className={styles.signUpLink}>
          Sign Up
        </a>
      </div>
    );
  }

  return (
    <div className={styles.userMenu}>
      <button
        className={styles.userButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="User menu"
      >
        <span className={styles.userIcon}>👤</span>
        <span className={styles.userEmail}>{userEmail || "User"}</span>
      </button>
      {isOpen && (
        <div className={styles.dropdown}>
          <div className={styles.dropdownItem}>
            <strong>Signed in as:</strong>
            <div>{userEmail}</div>
          </div>
          <hr className={styles.divider} />
          <button className={styles.logoutButton} onClick={handleLogout}>
            Sign Out
          </button>
        </div>
      )}
    </div>
  );
}
