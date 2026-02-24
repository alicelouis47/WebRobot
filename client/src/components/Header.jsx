import React from 'react';

function Header({ connected }) {
    return (
        <header className="header">
            <div className="logo">
                <div className="logo-icon">
                    <svg viewBox="0 0 100 100" className="robot-svg">
                        <circle cx="50" cy="30" r="15" className="head" />
                        <rect x="35" y="45" width="30" height="35" rx="5" className="body" />
                        <line x1="35" y1="55" x2="15" y2="70" className="arm left-arm" />
                        <line x1="65" y1="55" x2="85" y2="70" className="arm right-arm" />
                        <circle cx="15" cy="70" r="5" className="hand" />
                        <circle cx="85" cy="70" r="5" className="hand" />
                    </svg>
                </div>
                <h1>ROBOT ARM <span className="highlight">CONTROL</span></h1>
            </div>
            <div className={`connection-status ${connected ? 'connected' : ''}`} id="connectionStatus">
                <span className="status-dot"></span>
                <span className="status-text">{connected ? 'Connected' : 'Disconnected'}</span>
            </div>
        </header>
    );
}

export default Header;
