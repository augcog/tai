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
<<<<<<< HEAD:ai_course_bot/ai-chatbot/components/header.tsx
<<<<<<< HEAD
import { SelectCourse } from '@/components/select-course'
=======
import { LoginButton } from '@/components/login-button'
=======
import { LoginButton } from '@/tai/components/login-button'
>>>>>>> def647e43f040b8c085af99271a302bba05d9083:ai_course_bot/ai-chatbot/app/core/components/header.tsx
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
<<<<<<< HEAD:ai_course_bot/ai-chatbot/components/header.tsx
<<<<<<< HEAD
        </>
      ) : (
        <Link href="/" target="_blank" rel="nofollow">
          <IconNextChat className="size-6 mr-2 dark:hidden" inverted />
          <IconNextChat className="hidden size-6 mr-2 dark:block" />
        </Link>
=======

=======
>>>>>>> def647e43f040b8c085af99271a302bba05d9083:ai_course_bot/ai-chatbot/app/core/components/header.tsx
        </>
      ) : (
        <Link href="/" target="_blank" rel="nofollow">
          {/* <Image src="/TAI_prompt.png" alt="logo" width={50} height={50} /> */}
          {/* <IconNextChat className="size-6 mr-2 dark:hidden" inverted />
          <IconNextChat className="hidden size-6 mr-2 dark:block" /> */}
<<<<<<< HEAD:ai_course_bot/ai-chatbot/components/header.tsx
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
=======
        </Link>
      )}
      <div className="flex items-center">
        <IconSeparator className="size-6 text-muted-foreground/50" />
        {session?.user ? <UserMenu user={session.user} /> : <LoginButton />}
>>>>>>> def647e43f040b8c085af99271a302bba05d9083:ai_course_bot/ai-chatbot/app/core/components/header.tsx
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
<<<<<<< HEAD:ai_course_bot/ai-chatbot/components/header.tsx

>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
=======
>>>>>>> def647e43f040b8c085af99271a302bba05d9083:ai_course_bot/ai-chatbot/app/core/components/header.tsx
    </header>
  )
}
