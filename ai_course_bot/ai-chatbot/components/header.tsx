import * as React from 'react'
import Link from 'next/link'
import { cn } from '@/lib/utils'
import { auth } from '@/auth'
import { Button, buttonVariants } from '@/components/ui/button'
import {
  IconNextChat,
  IconSeparator,
} from '@/components/ui/icons'
import { UserMenu } from '@/components/user-menu'
import { SidebarMobile } from './sidebar-mobile'
import { SidebarToggle } from './sidebar-toggle'
import { ChatHistory } from './chat-history'
<<<<<<< HEAD
import { SelectCourse } from '@/components/select-course'
=======
import { LoginButton } from '@/components/login-button'
import Image from 'next/image'
>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc

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
<<<<<<< HEAD
        </>
      ) : (
        <Link href="/" target="_blank" rel="nofollow">
          <IconNextChat className="size-6 mr-2 dark:hidden" inverted />
          <IconNextChat className="hidden size-6 mr-2 dark:block" />
        </Link>
=======

        </>
      ) : (
          
          <Link href="/" target="_blank" rel="nofollow">
            {/* <Image src="/TAI_prompt.png" alt="logo" width={50} height={50} /> */}
          {/* <IconNextChat className="size-6 mr-2 dark:hidden" inverted />
          <IconNextChat className="hidden size-6 mr-2 dark:block" /> */}
          </Link>
>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
      )}
      <div className="flex items-center">
        <IconSeparator className="size-6 text-muted-foreground/50" />
        {session?.user ? (
          <UserMenu user={session.user} />
        ) : (
<<<<<<< HEAD
          <Button variant="link" asChild className="-ml-2">
            <Link href="/sign-in?callbackUrl=/">Login</Link>
          </Button>
=======
          <LoginButton />
>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
        )}
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
<<<<<<< HEAD
        <SelectCourse />

      </div>
=======
        <Image src="/TAI_logo.png" alt="logo" width={40} height={40} />
      </div>

>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
    </header>
  )
}
