import React from 'react';

/**
 * Shopify-style "code in / output" two-pane block.
 *
 * Usage in MDX — pass exactly two fenced code blocks as children. The
 * first is the "input" (code), the second is the "output". Each block
 * keeps its own syntax highlighting via Nextra's shiki pipeline.
 *
 * <CodeDemo>
 * ```c filename="Code"
 * PWM_SetThrottle(0.5f);
 * ```
 *
 * ```text filename="Output"
 * [PWM] duty=1500us  (50% throttle)
 * ```
 * </CodeDemo>
 */
export function CodeDemo({ children }: { children: React.ReactNode }) {
  return <div className="magalhaes-code-demo">{children}</div>;
}

export default CodeDemo;
