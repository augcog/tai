'use client'

import * as React from 'react'
import { ThemeProvider as NextThemesProvider } from 'next-themes'
import { ThemeProviderProps } from 'next-themes/dist/types'
import { SidebarProvider } from '@/app/lib/hooks/use-sidebar'
import { TooltipProvider } from '@/app/components/ui/tooltip'
import { SessionProvider } from 'next-auth/react'

export function Providers({ children, ...props }: ThemeProviderProps) {
  return (
    <SessionProvider>
      <NextThemesProvider {...props} defaultTheme="light" themes={['light']}>
        <SidebarProvider>
          <TooltipProvider>{children}</TooltipProvider>
        </SidebarProvider>
      </NextThemesProvider>
    </SessionProvider>
  )
}
