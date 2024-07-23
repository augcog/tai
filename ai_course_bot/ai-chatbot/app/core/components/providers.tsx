'use client'

import * as React from 'react'
import { ThemeProvider as NextThemesProvider } from 'next-themes'
import { ThemeProviderProps } from 'next-themes/dist/types'
<<<<<<< HEAD:ai_course_bot/ai-chatbot/components/providers.tsx
import { SidebarProvider } from '@/lib/hooks/use-sidebar'
import { TooltipProvider } from '@/components/ui/tooltip'
<<<<<<< HEAD

export function Providers({ children, ...props }: ThemeProviderProps) {
  return (
    <NextThemesProvider {...props}>
      <SidebarProvider>
        <TooltipProvider>{children}</TooltipProvider>
      </SidebarProvider>
    </NextThemesProvider>
=======
import { SessionProvider } from "next-auth/react"
=======
import { SidebarProvider } from '@/tai/lib/hooks/use-sidebar'
import { TooltipProvider } from '@/tai/components/ui/tooltip'
import { SessionProvider } from 'next-auth/react'
>>>>>>> def647e43f040b8c085af99271a302bba05d9083:ai_course_bot/ai-chatbot/app/core/components/providers.tsx

export function Providers({ children, ...props }: ThemeProviderProps) {
  return (
    <SessionProvider>
      <NextThemesProvider {...props} defaultTheme="light" themes={['light']}>
        <SidebarProvider>
          <TooltipProvider>{children}</TooltipProvider>
        </SidebarProvider>
      </NextThemesProvider>
    </SessionProvider>
>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
  )
}
