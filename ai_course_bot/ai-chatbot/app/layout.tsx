import { Toaster } from 'react-hot-toast'
import { GeistSans } from 'geist/font/sans'
import { GeistMono } from 'geist/font/mono'

import '@/app/globals.css'
import { cn } from '@/tai/lib/utils'
import { TailwindIndicator } from '@/tai/components/tailwind-indicator'
import { Providers } from '@/tai/components/providers'
import { Header } from '@/tai/components/header'
import { SidebarDesktop } from '@/tai/components/sidebar-desktop'

export const metadata = {
  metadataBase: new URL(`https://${process.env.VERCEL_URL}`),
  title: {
    default: 'Course AI Chatbot',
    template: `%s - Course AI Chatbot`
  },
  description:
    'An AI-powered chatbot built with Next.js and Vercel for college course helps.',
  icons: {
<<<<<<< HEAD
    icon: '/favicon.ico',
    shortcut: '/favicon-16x16.png',
    apple: '/apple-touch-icon.png'
=======
    icon: '/TAI_prompt.png',
    shortcut: '/TAI_prompt.png',
    apple: '/TAI_prompt.png'
>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
  }
}

export const viewport = {
  themeColor: [
<<<<<<< HEAD
    { media: '(prefers-color-scheme: light)', color: 'white' },
<<<<<<< HEAD
    { media: '(prefers-color-scheme: dark)', color: 'black' }
=======
=======
    { media: '(prefers-color-scheme: light)', color: 'white' }
>>>>>>> def647e43f040b8c085af99271a302bba05d9083
    // { media: '(prefers-color-scheme: dark)', color: 'black' }
>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
  ]
}

interface RootLayoutProps {
  children: React.ReactNode
}

export default function RootLayout({ children }: RootLayoutProps) {
  return (
    <html lang="en" suppressHydrationWarning={true}>
      <body
        suppressHydrationWarning={true}
        className={cn(
          'font-sans antialiased',
          GeistSans.variable,
          GeistMono.variable
        )}
      >
        <Toaster />
        <Providers
          attribute="class"
          defaultTheme="system"
          enableSystem
          disableTransitionOnChange
        >
          <div className="flex flex-col min-h-screen">
            <Header />
            <main className="flex flex-col flex-1 bg-muted/50">
              <div className="relative flex h-[calc(100vh_-_theme(spacing.16))] overflow-hidden">
                <SidebarDesktop />
                <div className="group w-full overflow-auto pl-0 animate-in duration-300 ease-in-out peer-[[data-state=open]]:lg:pl-[250px] peer-[[data-state=open]]:xl:pl-[300px]">
                  {children}
                </div>
              </div>
            </main>
          </div>
          <TailwindIndicator />
        </Providers>
      </body>
    </html>
  )
}
