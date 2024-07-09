import { auth } from '@/auth'
import { LoginButton } from '@/components/login-button'
import { redirect } from 'next/navigation'
<<<<<<< HEAD

export default async function SignInPage() {
  const session = await auth()
=======
import { Chat } from '@/components/chat'
import { nanoid } from '@/lib/utils'

export default async function SignInPage() {
  const session = await auth()
  const id = nanoid()

>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
  // redirect to home if user is already logged in
  if (session?.user) {
    redirect('/')
  }

  return (
<<<<<<< HEAD
    <div className="flex h-[calc(100vh-theme(spacing.16))] items-center justify-center py-10">
      <LoginButton />
    </div>
=======
    // <p>hi</p>
    <Chat id={id} />
    // <LoginButton />

>>>>>>> cdbc2f5496cabb88b8715e1213624541579ec1fc
  )
}
