import React from 'react'

import { cn } from '@/lib/utils'
import { ExternalLink } from '@/components/external-link'

export function FooterText({ className, ...props }: React.ComponentProps<'p'>) {
  return (
    <p
      className={cn(
        'px-2 text-center text-xs leading-normal text-muted-foreground',
        className
      )}
      {...props}
    >
      Made with ❤️ by{' '}
      <ExternalLink href="https://www.nimbus-nova.com/">
        Nimbus Nova LLC
      </ExternalLink>{' '}
      and{' '}
      <ExternalLink href="https://vivecenter.berkeley.edu/">
        FHL Vive Center
      </ExternalLink>
    </p>
  )
}
