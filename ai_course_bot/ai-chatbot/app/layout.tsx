import { Toaster } from 'react-hot-toast'
import { GeistSans } from 'geist/font/sans'
import { GeistMono } from 'geist/font/mono'

import '@/app/globals.css'
import { cn } from '@/lib/utils'
import { TailwindIndicator } from '@/components/tailwind-indicator'
import { Providers } from '@/components/providers'
import { Header } from '@/components/header'

export const metadata = {
  metadataBase: new URL(`https://${process.env.VERCEL_URL}`),
  title: {
    default: 'Course AI Chatbot',
    template: `%s - Course AI Chatbot`
  },
  description: 'An AI-powered chatbot built with Next.js and Vercel for college course helps.',
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
    { media: '(prefers-color-scheme: light)', color: 'white' },
<<<<<<< HEAD
    { media: '(prefers-color-scheme: dark)', color: 'black' }
=======
    // { media: '(prefers-color-scheme: dark)', color: 'black' }
>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
  ]
}

interface RootLayoutProps {
  children: React.ReactNode
}

export default function RootLayout({ children }: RootLayoutProps) {
  return (
    <html lang="en" suppressHydrationWarning>
      <body
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
            <main className="flex flex-col flex-1 bg-muted/50">{children}</main>
          </div>
          <TailwindIndicator />
        </Providers>
      </body>
    </html>
  )
}
