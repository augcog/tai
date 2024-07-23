import * as React from 'react'

import Link from 'next/link'

<<<<<<< HEAD:ai_course_bot/ai-chatbot/components/chat-history.tsx
import { cn } from '@/lib/utils'
import { SidebarList } from '@/components/sidebar-list'
import { buttonVariants } from '@/components/ui/button'
import { IconPlus } from '@/components/ui/icons'
<<<<<<< HEAD

=======
>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
=======
import { cn } from '@/tai/lib/utils'
import { SidebarList } from '@/tai/components/sidebar-list'
import { buttonVariants } from '@/tai/components/ui/button'
import { IconPlus } from '@/tai/components/ui/icons'
>>>>>>> def647e43f040b8c085af99271a302bba05d9083:ai_course_bot/ai-chatbot/app/core/components/chat-history.tsx
interface ChatHistoryProps {
  userId?: string
}

export async function ChatHistory({ userId }: ChatHistoryProps) {
  return (
    <div className="flex flex-col h-full">
      <div className="px-2 my-4">
<<<<<<< HEAD:ai_course_bot/ai-chatbot/components/chat-history.tsx
<<<<<<< HEAD
=======

>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
=======
>>>>>>> def647e43f040b8c085af99271a302bba05d9083:ai_course_bot/ai-chatbot/app/core/components/chat-history.tsx
        <Link
          href="/"
          className={cn(
            buttonVariants({ variant: 'outline' }),
            'h-10 w-full justify-start bg-zinc-50 px-4 shadow-none transition-colors hover:bg-zinc-200/40 dark:bg-zinc-900 dark:hover:bg-zinc-300/10'
          )}
        >
<<<<<<< HEAD:ai_course_bot/ai-chatbot/components/chat-history.tsx
<<<<<<< HEAD
          <IconPlus className="-translate-x-2 stroke-2" />
          New Chat
=======
            <IconPlus className="-translate-x-2 stroke-2" />
            New Chat
>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
=======
          <IconPlus className="-translate-x-2 stroke-2" />
          New Chat
>>>>>>> def647e43f040b8c085af99271a302bba05d9083:ai_course_bot/ai-chatbot/app/core/components/chat-history.tsx
        </Link>
      </div>
      <React.Suspense
        fallback={
          <div className="flex flex-col flex-1 px-4 space-y-4 overflow-auto">
            {Array.from({ length: 10 }).map((_, i) => (
              <div
                key={i}
                className="w-full h-6 rounded-md shrink-0 animate-pulse bg-zinc-200 dark:bg-zinc-800"
              />
            ))}
          </div>
        }
      >
        {/* @ts-ignore */}
        <SidebarList userId={userId} />
      </React.Suspense>
    </div>
  )
}
