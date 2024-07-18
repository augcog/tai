import * as React from 'react'
import Link from 'next/link'
import { cn } from '@/tai/lib/utils'
import { auth } from '@/tai/utils/auth'
import { Button, buttonVariants } from '@/tai/components/ui/button'
import { IconNextChat, IconSeparator } from '@/tai/components/ui/icons'
import { UserMenu } from '@/tai/components/user-menu'
import { SidebarMobile } from './sidebar-mobile'
import { SidebarToggle } from './sidebar-toggle'
import { ChatHistory } from './chat-history'
import { LoginButton } from '@/tai/components/login-button'
import Image from 'next/image'

async function UserOrLogin() {
  const session = await auth()
  return (
    <>
      {session?.user ? (
        <>
          <SidebarMobile>
            <ChatHistory userId={session.user.id} />
          </SidebarMobile>
          <SidebarToggle />
        </>
      ) : (
        <Link href="/" target="_blank" rel="nofollow">
          {/* <Image src="/TAI_prompt.png" alt="logo" width={50} height={50} /> */}
          {/* <IconNextChat className="size-6 mr-2 dark:hidden" inverted />
          <IconNextChat className="hidden size-6 mr-2 dark:block" /> */}
        </Link>
      )}
      <div className="flex items-center">
        <IconSeparator className="size-6 text-muted-foreground/50" />
        {session?.user ? <UserMenu user={session.user} /> : <LoginButton />}
      </div>
    </>
  )
}

export function Header() {
  return (
    <header className="sticky top-0 z-50 flex items-center justify-between w-full h-16 px-4 border-b shrink-0 bg-gradient-to-b from-background/10 via-background/50 to-background/80 backdrop-blur-xl">
      <div className="flex items-center">
        <React.Suspense fallback={<div className="flex-1 overflow-auto" />}>
          <UserOrLogin />
        </React.Suspense>
      </div>
      <div className="flex items-center justify-end space-x-2">
        <Image src="/TAI_logo.png" alt="logo" width={40} height={40} />
      </div>
    </header>
  )
}
