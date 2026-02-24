import React from 'react';

function QuickActions({ onAction }) {
    return (
        <section className="actions-panel glass-panel">
            <h2><span className="icon">⚡</span> Quick Actions</h2>
            <div className="action-buttons">
                <button className="btn-action btn-home" onClick={() => onAction('home')}>
                    <span className="btn-icon">🏠</span>
                    <span className="btn-text">HOME</span>
                </button>
                <button className="btn-action btn-grab" onClick={() => onAction('grab')}>
                    <span className="btn-icon">✊</span>
                    <span className="btn-text">GRAB</span>
                </button>
                <button className="btn-action btn-release" onClick={() => onAction('release')}>
                    <span className="btn-icon">🖐️</span>
                    <span className="btn-text">RELEASE</span>
                </button>
                <button className="btn-action btn-stop" onClick={() => onAction('stop')}>
                    <span className="btn-icon">🛑</span>
                    <span className="btn-text">STOP</span>
                </button>
            </div>
        </section>
    );
}

export default QuickActions;
